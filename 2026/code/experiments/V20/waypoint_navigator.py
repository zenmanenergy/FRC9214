"""Waypoint Navigator - Stage-based autonomous navigation system"""
import math
import time


class WaypointNavigator:
	"""
	Navigate robot through waypoints using a stage-based system:
	- Stage 0: Initialize
	- Stage 1: Rotate to target
	- Stage 2: Move forward
	- Stage 3: Complete (move to next waypoint)
	"""
	
	def __init__(self, swerve_drive):
		"""
		Initialize navigator.
		
		Args:
			swerve_drive: SwerveDrive instance
		"""
		self.drive = swerve_drive
		self.waypoints = []
		self.current_waypoint_index = 0
		self.stage = 0  # 0=idle, 1=rotate, 2=move, 3=done
		self.is_active = False
		self.start_time = None
		
		# Tuning parameters
		self.rotation_tolerance = 5.0  # degrees
		self.position_tolerance = 10.0  # cm
		self.max_rotation_speed = 0.8  # power 0-1 (increased from 0.5 for faster rotation)
		self.max_move_speed = 0.8  # power 0-1
		self.timeout_per_stage = 10.0  # seconds
	
	def set_waypoints(self, waypoints):
		"""
		Set list of waypoints.
		
		Args:
			waypoints: List of dicts with 'x', 'y' keys (in cm)
		"""
		self.waypoints = waypoints
		self.current_waypoint_index = 0
		self.stage = 0
		print(f"[NAVIGATOR] Set {len(waypoints)} waypoints")
	
	def start(self):
		"""Start navigation sequence"""
		if not self.waypoints:
			print("[NAVIGATOR] No waypoints set!")
			return False
		
		self.is_active = True
		self.current_waypoint_index = 0
		self.stage = 1  # Start with rotation stage
		self.start_time = time.time()
		print(f"[NAVIGATOR] Starting: {len(self.waypoints)} waypoints")
		return True
	
	def stop(self):
		"""Stop navigation and stop robot"""
		self.is_active = False
		self.stage = 0
		self.drive.stop_all()
		print("[NAVIGATOR] Stopped")
	
	def is_finished(self):
		"""Check if all waypoints completed"""
		return (self.current_waypoint_index >= len(self.waypoints)) and not self.is_active
	
	def get_status(self):
		"""Get current navigation status"""
		if not self.is_active:
			return {
				"active": False,
				"stage": 0,
				"waypoint": 0,
				"total": len(self.waypoints),
				"progress": "Idle"
			}
		
		current_wp = self.waypoints[self.current_waypoint_index]
		return {
			"active": True,
			"stage": self.stage,
			"waypoint": self.current_waypoint_index + 1,
			"total": len(self.waypoints),
			"target": current_wp,
			"current_x": self.drive.odometry.get_x(),
			"current_y": self.drive.odometry.get_y(),
			"heading": self.drive.odometry.get_heading(),
			"progress": f"Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} Stage {self.stage}"
		}
	
	def update(self):
		"""Call periodically (e.g., from robot loop) to execute navigation"""
		if not self.is_active:
			return
		
		# Get current pose
		current_x = self.drive.odometry.get_x()
		current_y = self.drive.odometry.get_y()
		current_heading = self.drive.odometry.get_heading()
		
		# Get target waypoint
		target = self.waypoints[self.current_waypoint_index]
		target_x = target["x"]
		target_y = target["y"]
		target_heading = target.get("heading", None)  # Use specified heading if provided
		
		# Calculate distance and angle to target
		dx = target_x - current_x
		dy = target_y - current_y
		distance_to_target = math.sqrt(dx**2 + dy**2)
		
		# Use specified heading if provided, otherwise calculate angle to target
		if target_heading is not None:
			target_angle = target_heading
		else:
			target_angle = math.degrees(math.atan2(dy, dx))
		
		# Normalize angles to 0-360
		target_angle = target_angle % 360
		current_heading = current_heading % 360
		
		# Calculate shortest rotation
		angle_diff = target_angle - current_heading
		if angle_diff > 180:
			angle_diff -= 360
		elif angle_diff < -180:
			angle_diff += 360
		
		elapsed = time.time() - self.start_time
		
		# ===== STAGE 1: ROTATE TO TARGET =====
		if self.stage == 1:
			if abs(angle_diff) < self.rotation_tolerance:
				# Rotation complete, move to stage 2
				self.stage = 2
				self.start_time = time.time()
				print(f"[NAVIGATOR] WP {self.current_waypoint_index + 1}: Rotation complete, moving")
			elif elapsed > self.timeout_per_stage:
				# Timeout - skip waypoint
				print(f"[NAVIGATOR] WP {self.current_waypoint_index + 1}: Rotation timeout (target={target_angle:.1f}° current={current_heading:.1f}° diff={angle_diff:.1f}°)")
				self._advance_waypoint()
			else:
				# Rotate toward target
				rotation_power = (angle_diff / 180.0) * self.max_rotation_speed
				rotation_power = max(-self.max_rotation_speed, min(self.max_rotation_speed, rotation_power))
				print(f"[NAVIGATOR] Stage 1 - Target {target_angle:.1f}° Current {current_heading:.1f}° Diff {angle_diff:.1f}° Power {rotation_power:.3f} Elapsed {elapsed:.2f}s")
				self.drive.rotate_in_place(rotation_power)
		
		# ===== STAGE 2: MOVE TO TARGET =====
		elif self.stage == 2:
			if distance_to_target < self.position_tolerance:
				# Reached waypoint
				self.drive.stop_all()
				self.stage = 3
				self.start_time = time.time()
				print(f"[NAVIGATOR] WP {self.current_waypoint_index + 1}: Reached target")
			elif elapsed > self.timeout_per_stage:
				# Timeout - skip waypoint
				print(f"[NAVIGATOR] WP {self.current_waypoint_index + 1}: Movement timeout")
				self._advance_waypoint()
			else:
				# Move toward target
				forward_power = min(self.max_move_speed, distance_to_target / 200.0)
				# Also correct angle drift
				rotation_correction = (angle_diff / 45.0) * 0.2  # gentle correction
				self.drive.drive_swerve(forward_power, 0, rotation_correction)
		
		# ===== STAGE 3: DWELL / ADVANCE =====
		elif self.stage == 3:
			dwell_time = target.get("dwell", 0.5)  # Optional dwell at waypoint
			if elapsed > dwell_time:
				self._advance_waypoint()
	
	def _advance_waypoint(self):
		"""Move to next waypoint or finish"""
		self.current_waypoint_index += 1
		
		if self.current_waypoint_index >= len(self.waypoints):
			# All waypoints complete
			self.drive.stop_all()
			self.is_active = False
			self.stage = 0
			print(f"[NAVIGATOR] Navigation complete!")
		else:
			# Next waypoint
			self.stage = 1
			self.start_time = time.time()
			next_wp = self.waypoints[self.current_waypoint_index]
			print(f"[NAVIGATOR] Moving to waypoint {self.current_waypoint_index + 1}: ({next_wp['x']}, {next_wp['y']})")
