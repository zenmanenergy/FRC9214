import cv2
import numpy as np
import math
from sklearn.cluster import DBSCAN
from networktables import NetworkTables


class BallClusterVision:
	def __init__(self, server_ip, enable_debug=True):
		# ---------------- NetworkTables ----------------
		NetworkTables.initialize(server=server_ip)
		self.nt = NetworkTables.getTable("vision")

		# ---------------- Camera / Stereo Params ----------------
		self.image_width = 640
		self.cx = self.image_width / 2.0
		self.fx = 700.0				# pixels (CALIBRATE)
		self.baseline = 0.60			# meters

		# ---------------- Ball Detection Params ----------------
		self.yellow_lower = np.array([20, 100, 100])
		self.yellow_upper = np.array([35, 255, 255])

		self.min_radius_px = 6
		self.max_radius_px = 200
		self.min_circularity = 0.6

		# ---------------- Range Limits ----------------
		self.min_range = 0.4
		self.max_range = 6.0

		# ---------------- Clustering ----------------
		self.cluster_eps = 0.35			# meters
		self.cluster_min = 6

		# ---------------- Temporal Smoothing ----------------
		self.alpha = 0.3
		self.smoothed_x = None
		self.smoothed_y = None
		self.smoothed_heading_vec = None

		# ---------------- Cluster Locking ----------------
		self.locked_cluster = None		# (forward, left)
		self.locked_confidence = 0
		self.frames_since_seen = 0
		self.max_missing_frames = 10
		self.min_confidence = 5
		
		# ---------------- Debug Visualization ----------------
		self.enable_debug = enable_debug
		self.debug_stats = {
			'left_balls': 0,
			'right_balls': 0,
			'stereo_matches': 0,
			'valid_points': 0,
			'clusters': 0,
			'largest_cluster_size': 0,
			'locked_confidence': 0,
			'frames_since_seen': 0
		}

	# ---------------------------------------------------------
	# Robot pose from RoboRIO
	# ---------------------------------------------------------
	def _get_robot_pose(self):
		x = self.nt.getNumber("robot/x", None)
		y = self.nt.getNumber("robot/y", None)
		h = self.nt.getNumber("robot/heading", None)

		if x is None or y is None or h is None:
			return None

		return x, y, h

	# ---------------------------------------------------------
	# Ball detection
	# ---------------------------------------------------------
	def _detect_balls(self, frame):
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
		mask = cv2.medianBlur(mask, 7)

		contours, _ = cv2.findContours(
			mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
		)

		centers = []

		for c in contours:
			(x, y), r = cv2.minEnclosingCircle(c)

			if r < self.min_radius_px or r > self.max_radius_px:
				continue

			area = cv2.contourArea(c)
			circle_area = math.pi * r * r
			if circle_area <= 0:
				continue

			if area / circle_area < self.min_circularity:
				continue

			centers.append((int(x), int(y)))

		return centers, mask

	# ---------------------------------------------------------
	# Match left/right balls by scanline
	# ---------------------------------------------------------
	def _match_stereo(self, left_pts, right_pts, max_y_diff=10):
		matches = []

		for lx, ly in left_pts:
			best_rx = None
			best_dy = max_y_diff

			for rx, ry in right_pts:
				dy = abs(ly - ry)
				if dy < best_dy:
					best_dy = dy
					best_rx = rx

			if best_rx is not None:
				matches.append((lx, best_rx))

		return matches

	# ---------------------------------------------------------
	# Stereo disparity → robot-frame point
	# Robot frame: +X forward, +Y left
	# ---------------------------------------------------------
	def _stereo_to_point(self, lx, rx):
		disparity = lx - rx
		if abs(disparity) < 2:
			return None

		Z = (self.fx * self.baseline) / disparity
		if Z < self.min_range or Z > self.max_range:
			return None

		Y = Z * (lx - self.cx) / self.fx
		return Z, Y

	# ---------------------------------------------------------
	# NetworkTables helpers
	# ---------------------------------------------------------
	def _publish_invalid(self):
		self.nt.putBoolean("target/valid", False)
		self.nt.putNumber("target/confidence", 0)

	def _publish_target(self, x, y, heading, confidence):
		self.nt.putNumber("target/x", x)
		self.nt.putNumber("target/y", y)
		self.nt.putNumber("target/heading", heading)
		self.nt.putNumber("target/confidence", confidence)
		self.nt.putBoolean("target/valid", True)

	# ---------------------------------------------------------
	# Debug visualization
	# ---------------------------------------------------------
	def _draw_detection_debug(self, left_frame, right_frame, left_pts, right_pts, matches):
		"""Draw detection and matching debug visualization."""
		left_debug = left_frame.copy()
		right_debug = right_frame.copy()
		
		# Draw detected balls
		for x, y in left_pts:
			cv2.circle(left_debug, (x, y), 8, (0, 255, 0), 2)
			cv2.circle(left_debug, (x, y), 3, (0, 255, 0), -1)
		
		for x, y in right_pts:
			cv2.circle(right_debug, (x, y), 8, (0, 255, 0), 2)
			cv2.circle(right_debug, (x, y), 3, (0, 255, 0), -1)
		
		# Draw matches with lines
		match_debug = np.hstack([left_debug, right_debug])
		for lx, rx in matches:
			# Find y-coordinate (should be similar for both)
			ly = next((y for x, y in left_pts if x == lx), 0)
			ry = next((y for x, y in right_pts if x == rx), 0)
			
			cv2.line(match_debug, (lx, ly), (left_frame.shape[1] + rx, ry), (255, 0, 0), 2)
			cv2.circle(match_debug, (lx, ly), 5, (0, 0, 255), -1)
			cv2.circle(match_debug, (left_frame.shape[1] + rx, ry), 5, (0, 0, 255), -1)
		
		# Add stats
		cv2.putText(match_debug, f"Left: {len(left_pts)} | Right: {len(right_pts)} | Matches: {len(matches)}", 
					(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
		
		return match_debug
	
	def _draw_clustering_debug(self, points, labels, largest_label=None):
		"""Create a 2D plot visualization of 3D points (Top-down view)."""
		# Create a blank canvas (forward x depth: 0-6m, left y width: -2 to 2m)
		canvas_height = 600  # 6 meters forward
		canvas_width = 400   # 4 meters left/right
		
		debug_img = np.ones((canvas_height, canvas_width, 3), dtype=np.uint8) * 255
		
		# Draw grid
		cv2.line(debug_img, (canvas_width//2, 0), (canvas_width//2, canvas_height), (200, 200, 200), 1)
		for i in range(0, canvas_height, 100):
			cv2.putText(debug_img, f"{i//100}m", (5, i+20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
		
		# Draw points
		for i, point in enumerate(points):
			z, y = point  # forward, left
			px = int(canvas_width//2 + (y / 3.0) * (canvas_width//2))  # left = ±3m
			py = int((z / 6.0) * canvas_height)  # forward = 0-6m
			
			if 0 <= px < canvas_width and 0 <= py < canvas_height:
				label = labels[i]
				color = (0, 255, 0) if label == -1 else (255, 100, int(100 + 155 * (label % 10) / 10))
				if largest_label is not None and label == largest_label:
					cv2.circle(debug_img, (px, py), 8, (0, 0, 255), -1)
					cv2.circle(debug_img, (px, py), 8, (255, 255, 255), 2)
				else:
					cv2.circle(debug_img, (px, py), 5, color, -1)
		
		cv2.putText(debug_img, "Top-down view (Robot at bottom)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
		cv2.putText(debug_img, "Red = Target Cluster | Green = Outliers", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
		
		return debug_img
	
	def _draw_stats_panel(self):
		"""Create a statistics panel."""
		panel_height = 250
		panel_width = 400
		panel = np.ones((panel_height, panel_width, 3), dtype=np.uint8) * 240
		
		y_pos = 25
		line_height = 25
		
		stats_text = [
			f"Left Balls: {self.debug_stats['left_balls']}",
			f"Right Balls: {self.debug_stats['right_balls']}",
			f"Stereo Matches: {self.debug_stats['stereo_matches']}",
			f"Valid Points: {self.debug_stats['valid_points']}",
			f"Clusters Found: {self.debug_stats['clusters']}",
			f"Largest Cluster: {self.debug_stats['largest_cluster_size']}",
			f"Locked Confidence: {self.debug_stats['locked_confidence']}",
			f"Frames Missing: {self.debug_stats['frames_since_seen']}",
		]
		
		for text in stats_text:
			cv2.putText(panel, text, (15, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
			y_pos += line_height
		
		return panel

	# ---------------------------------------------------------
	# MAIN UPDATE
	# ---------------------------------------------------------
	def update(self, left_frame, right_frame):
		robot_pose = self._get_robot_pose()
		if robot_pose is None:
			self._publish_invalid()
			if self.enable_debug:
				print("[DEBUG] No robot pose available")
			return

		# --- Detect balls ---
		left_pts, left_mask = self._detect_balls(left_frame)
		right_pts, right_mask = self._detect_balls(right_frame)
		
		self.debug_stats['left_balls'] = len(left_pts)
		self.debug_stats['right_balls'] = len(right_pts)

		if len(left_pts) == 0 or len(right_pts) == 0:
			self._publish_invalid()
			if self.enable_debug:
				print(f"[DEBUG] Insufficient balls detected - Left: {len(left_pts)}, Right: {len(right_pts)}")
			return

		# --- Stereo matching ---
		matches = self._match_stereo(left_pts, right_pts)
		self.debug_stats['stereo_matches'] = len(matches)
		
		if len(matches) < self.cluster_min:
			self._publish_invalid()
			if self.enable_debug:
				print(f"[DEBUG] Not enough stereo matches: {len(matches)} < {self.cluster_min}")
			return

		# --- Stereo points ---
		points = []
		for lx, rx in matches:
			p = self._stereo_to_point(lx, rx)
			if p is not None:
				points.append(p)

		self.debug_stats['valid_points'] = len(points)
		
		if len(points) < self.cluster_min:
			self._publish_invalid()
			if self.enable_debug:
				print(f"[DEBUG] Not enough valid stereo points: {len(points)} < {self.cluster_min}")
			return

		# --- Clustering ---
		X = np.array(points)
		db = DBSCAN(
			eps=self.cluster_eps,
			min_samples=self.cluster_min
		).fit(X)

		labels = db.labels_
		valid = labels[labels != -1]
		
		self.debug_stats['clusters'] = len(set(valid)) if len(valid) > 0 else 0

		if len(valid) == 0:
			self._publish_invalid()
			if self.enable_debug:
				print("[DEBUG] No valid clusters found")
			return

		largest_label = max(set(valid), key=list(valid).count)
		cluster = X[labels == largest_label]
		
		self.debug_stats['largest_cluster_size'] = len(cluster)

		# --- New cluster centroid (robot frame) ---
		new_cx = np.mean(cluster[:, 0])		# forward
		new_cy = np.mean(cluster[:, 1])		# left
		new_conf = len(cluster)

		# --- Cluster locking ---
		if self.locked_cluster is None:
			self.locked_cluster = (new_cx, new_cy)
			self.locked_confidence = new_conf
			self.frames_since_seen = 0
		else:
			dist = math.hypot(
				new_cx - self.locked_cluster[0],
				new_cy - self.locked_cluster[1]
			)

			if dist < 0.75:
				self.locked_cluster = (new_cx, new_cy)
				self.locked_confidence = new_conf
				self.frames_since_seen = 0
			else:
				self.frames_since_seen += 1

		self.debug_stats['locked_confidence'] = self.locked_confidence
		self.debug_stats['frames_since_seen'] = self.frames_since_seen

		if (
			self.frames_since_seen > self.max_missing_frames
			or self.locked_confidence < self.min_confidence
		):
			self.locked_cluster = None
			self._publish_invalid()
			if self.enable_debug:
				print(f"[DEBUG] Cluster lock lost - Frames: {self.frames_since_seen}, Confidence: {self.locked_confidence}")
			return

		cx, cy = self.locked_cluster

		# --- Convert to field frame ---
		rx, ry, rh = robot_pose

		tx = rx + math.cos(rh)*cx - math.sin(rh)*cy
		ty = ry + math.sin(rh)*cx + math.cos(rh)*cy
		raw_heading = math.atan2(ty - ry, tx - rx)

		# --- Temporal smoothing ---
		if self.smoothed_x is None:
			self.smoothed_x = tx
			self.smoothed_y = ty
			self.smoothed_heading_vec = (
				math.cos(raw_heading),
				math.sin(raw_heading)
			)
		else:
			self.smoothed_x = (
				self.alpha * tx +
				(1.0 - self.alpha) * self.smoothed_x
			)
			self.smoothed_y = (
				self.alpha * ty +
				(1.0 - self.alpha) * self.smoothed_y
			)

			hx, hy = self.smoothed_heading_vec
			hx = self.alpha * math.cos(raw_heading) + (1.0 - self.alpha) * hx
			hy = self.alpha * math.sin(raw_heading) + (1.0 - self.alpha) * hy

			n = math.hypot(hx, hy)
			self.smoothed_heading_vec = (hx / n, hy / n)

		smoothed_heading = math.atan2(
			self.smoothed_heading_vec[1],
			self.smoothed_heading_vec[0]
		)

		self._publish_target(
			self.smoothed_x,
			self.smoothed_y,
			smoothed_heading,
			self.locked_confidence
		)
	
	# ---------------------------------------------------------
	# Display debug visualization (call after update)
	# ---------------------------------------------------------
	def show_debug(self, left_frame, right_frame, left_pts, right_pts, matches, points=None, labels=None, largest_label=None):
		"""Display all debug visualizations."""
		if not self.enable_debug:
			return
		
		# Draw detection and stereo matching
		detection_debug = self._draw_detection_debug(left_frame, right_frame, left_pts, right_pts, matches)
		cv2.imshow("Stereo Detection & Matching", detection_debug)
		
		# Draw clustering
		if points is not None and labels is not None:
			clustering_debug = self._draw_clustering_debug(points, labels, largest_label)
			cv2.imshow("Clustering (Top-Down View)", clustering_debug)
		
		# Draw stats panel
		stats_panel = self._draw_stats_panel()
		cv2.imshow("Vision Stats", stats_panel)
		
		# Handle key presses
		key = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			return False
		return True
