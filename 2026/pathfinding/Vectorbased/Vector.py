import cv2
import numpy as np
import os
import json
import itertools
import math
from scipy.interpolate import CubicSpline
import heapq

###############################################################################
# LOADING / SAVING
###############################################################################
def load_obstacles(file_path):
	if os.path.exists(file_path):
		with open(file_path, 'r') as file:
			data = json.load(file)
			return data.get("obstacles", [])
	return []

def save_obstacles(file_path, obstacles):
	with open(file_path, 'w') as file:
		json.dump({"obstacles": obstacles}, file, indent=4)

###############################################################################
# GEOMETRY UTILS
###############################################################################
def is_visible(p1, p2, obstacles):
	for poly in obstacles:
		n = len(poly)
		for i in range(n):
			p3 = tuple(poly[i])
			p4 = tuple(poly[(i + 1) % n])
			# Only skip if the entire segment (p1, p2) exactly matches the polygon edge (p3, p4).
			if (p1 == p3 and p2 == p4) or (p1 == p4 and p2 == p3):
				continue
			if do_intersect(p1, p2, p3, p4):
				return False
	return True

def do_intersect(A, B, C, D):
	def orientation(p, q, r):
		val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
		if val == 0:
			return 0
		return 1 if val > 0 else 2

	def on_segment(p, q, r):
		return (
			min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
			min(p[1], r[1]) <= q[1] <= max(p[1], r[1])
		)

	o1 = orientation(A, B, C)
	o2 = orientation(A, B, D)
	o3 = orientation(C, D, A)
	o4 = orientation(C, D, B)

	if o1 != o2 and o3 != o4:
		return True
	if o1 == 0 and on_segment(A, C, B):
		return True
	if o2 == 0 and on_segment(A, D, B):
		return True
	if o3 == 0 and on_segment(C, A, D):
		return True
	if o4 == 0 and on_segment(C, B, D):
		return True
	return False

def euclidean_distance(p1, p2):
	return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

###############################################################################
# VISIBILITY GRAPH & PATHFINDING
###############################################################################
def build_visibility_graph(waypoints, obstacles):
	graph = {p: [] for p in waypoints}
	for p1, p2 in itertools.combinations(waypoints, 2):
		if is_visible(p1, p2, obstacles):
			dist = euclidean_distance(p1, p2)
			graph[p1].append((p2, dist))
			graph[p2].append((p1, dist))
	return graph

def dijkstra(graph, start, goal):
	if start not in graph or goal not in graph:
		return []
	pq = [(0, start)]
	distances = {point: float('inf') for point in graph}
	distances[start] = 0
	came_from = {}
	while pq:
		cost, current = heapq.heappop(pq)
		if current == goal:
			path = [goal]
			while current in came_from:
				current = came_from[current]
				path.append(current)
			return path[::-1]
		for neighbor, weight in graph[current]:
			new_cost = cost + weight
			if new_cost < distances[neighbor]:
				distances[neighbor] = new_cost
				came_from[neighbor] = current
				heapq.heappush(pq, (new_cost, neighbor))
	return []

###############################################################################
# PATH SMOOTHING & ROBOT DRAWING
###############################################################################
def smooth_path(waypoints):
	if len(waypoints) < 2:
		return waypoints
	x, y = zip(*waypoints)
	t = np.linspace(0, 1, len(waypoints))
	x_spline = CubicSpline(t, x)
	y_spline = CubicSpline(t, y)
	return [
		(int(x_spline(ti)), int(y_spline(ti)))
		for ti in np.linspace(0, 1, 50)
	]

def draw_robot(img, center, orientation, color=(255,0,0), radius=10):
	cv2.circle(img, (int(center[0]), int(center[1])), radius, color, 2)
	arrow_length = radius + 10
	end_x = int(center[0] + arrow_length * math.cos(orientation))
	end_y = int(center[1] + arrow_length * math.sin(orientation))
	cv2.arrowedLine(
		img,
		(int(center[0]), int(center[1])),
		(end_x, end_y),
		color,
		2,
		tipLength=0.3
	)

###############################################################################
# OPTIONAL: OUTER WAYPOINTS FOR PATH EXTENSION
###############################################################################
def add_outer_waypoints(obstacles, margin=40, subdiv=3):
	pts = []
	for poly in obstacles:
		xvals = [p[0] for p in poly]
		yvals = [p[1] for p in poly]
		minx, maxx = min(xvals), max(xvals)
		miny, maxy = min(yvals), max(yvals)
		bx1 = minx - margin
		bx2 = maxx + margin
		by1 = miny - margin
		by2 = maxy + margin
		box = [(bx1, by1), (bx2, by1), (bx2, by2), (bx1, by2)]
		for i in range(len(box)):
			p1 = box[i]
			p2 = box[(i + 1) % len(box)]
			pts.append(p1)
			for s in range(1, subdiv):
				f = s / subdiv
				mx = int(p1[0] + f*(p2[0] - p1[0]))
				my = int(p1[1] + f*(p2[1] - p1[1]))
				pts.append((mx, my))
	return pts

###############################################################################
# MAIN INTERACTION LOGIC
###############################################################################
def click_event(event, x, y, flags, param):
	"""
	Left-click logic:
	1) First click => set start if not set
	2) If start is set => each new click is a new "destination" appended to the chain.
	   We'll plan from the last point in the chain to this new destination.
	"""
	if event == cv2.EVENT_LBUTTONDOWN:
		if param["start"] is None:
			param["start"] = (x, y)
			cv2.circle(param["image"], param["start"], 5, (255,0,0), -1)
		else:
			new_dest = (x, y)
			param["destinations"].append(new_dest)
			# We plan from last waypoint to new_dest
			last_point = param["start"] if len(param["path_segments"]) == 0 else param["destinations"][-2]
			
			all_points = (
				param["obstacle_waypoints"]
				+ param["outer_waypoints"]
				+ [last_point, new_dest]
			)
			all_points = list(set(all_points))
			
			graph = build_visibility_graph(all_points, param["obstacles"])
			segment_path = dijkstra(graph, last_point, new_dest)

			if segment_path:
				if len(segment_path) > 1:
					sp = smooth_path(segment_path)
				else:
					sp = segment_path
				
				# Store for later "replay" usage
				param["path_segments"].append(sp)

				# Draw the final segment
				for i in range(len(sp) - 1):
					cv2.line(param["image"], sp[i], sp[i+1], (0, 255, 0), 2)

				# Optionally animate the last segment
				for i in range(len(sp) - 1):
					temp_img = param["image"].copy()
					heading = math.atan2(sp[i+1][1] - sp[i][1], sp[i+1][0] - sp[i][0])
					draw_robot(temp_img, sp[i], heading, (255,0,0), 12)
					cv2.imshow("Path Planning", temp_img)
					cv2.waitKey(50)
				
				# Draw final robot orientation
				if len(sp) >= 2:
					last_heading = math.atan2(sp[-1][1] - sp[-2][1], sp[-1][0] - sp[-2][0])
				else:
					last_heading = 0
				draw_robot(param["image"], sp[-1], last_heading, (255,0,0), 12)

			else:
				print("No valid path found for segment:", last_point, "->", new_dest)

		cv2.imshow("Path Planning", param["image"])

def replay_path(param):
	""" Clear the screen and redraw everything from scratch, animating. """
	# Reset to a clean copy
	display_img = param["base_image"].copy()

	# Redraw obstacles
	for poly in param["obstacles"]:
		pts = np.array(poly, np.int32).reshape((-1, 1, 2))
		cv2.polylines(display_img, [pts], True, (0, 0, 255), 2)

	# Draw start
	if param["start"] is not None:
		cv2.circle(display_img, param["start"], 5, (255, 0, 0), -1)
	
	# Go segment by segment
	for sp in param["path_segments"]:
		# Draw the lines
		for i in range(len(sp) - 1):
			cv2.line(display_img, sp[i], sp[i+1], (0, 255, 0), 2)
			temp_img = display_img.copy()
			heading = math.atan2(sp[i+1][1] - sp[i][1], sp[i+1][0] - sp[i][0])
			draw_robot(temp_img, sp[i], heading, (255,0,0), 12)
			cv2.imshow("Path Planning", temp_img)
			cv2.waitKey(50)
		# Draw final robot orientation at end of segment
		if len(sp) >= 2:
			last_heading = math.atan2(sp[-1][1] - sp[-2][1], sp[-1][0] - sp[-2][0])
		else:
			last_heading = 0
		draw_robot(display_img, sp[-1], last_heading, (255,0,0), 12)

	cv2.imshow("Path Planning", display_img)
	param["image"] = display_img  # update the param image

def reset_path(param):
	""" Clear all paths and destinations but keep obstacles and base image. """
	param["start"] = None
	param["destinations"] = []
	param["path_segments"] = []
	param["image"] = param["base_image"].copy()

	# Redraw obstacles on the new param["image"]
	for poly in param["obstacles"]:
		pts = np.array(poly, np.int32).reshape((-1, 1, 2))
		cv2.polylines(param["image"], [pts], True, (0, 0, 255), 2)
	cv2.imshow("Path Planning", param["image"])

def handle_keyboard(param):
	"""
	Listen for key presses to:
	  - 'r': reset path
	  - 'p': replay path
	  - 'q' or ESC: quit
	"""
	key = cv2.waitKey(10) & 0xFF
	if key == ord('r'):
		reset_path(param)
	elif key == ord('p'):
		replay_path(param)
	elif key == ord('q') or key == 27:  # ESC
		return True
	return False

###############################################################################
# MAIN
###############################################################################
if __name__ == "__main__":
	script_dir = os.path.dirname(os.path.abspath(__file__))
	obstacle_file = os.path.join(script_dir, "obstacles.json")
	image_path = os.path.join(script_dir, "REEFSCAPE2025.png")

	# Load base image
	base_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
	if base_image is None:
		raise ValueError("Image not found or unable to load.")

	# Load obstacles
	obstacles = load_obstacles(obstacle_file)

	# Draw obstacles onto a copy for the base "clean" image
	temp_img = base_image.copy()
	for poly in obstacles:
		pts = np.array(poly, np.int32).reshape((-1, 1, 2))
		cv2.polylines(temp_img, [pts], True, (0, 0, 255), 2)

	# Create obstacle waypoints (vertices + edge subdivisions)
	obstacle_waypoints = []
	num_segments = 3
	for poly in obstacles:
		n = len(poly)
		for i in range(n):
			p1 = poly[i]
			obstacle_waypoints.append(tuple(p1))
			p2 = poly[(i + 1) % n]
			for s in range(1, num_segments):
				f = s / num_segments
				mx = int(p1[0] + f*(p2[0] - p1[0]))
				my = int(p1[1] + f*(p2[1] - p1[1]))
				obstacle_waypoints.append((mx, my))
	obstacle_waypoints = list(set(obstacle_waypoints))

	# Optional: Add outer waypoints around obstacles
	outer_waypoints = add_outer_waypoints(obstacles, margin=40, subdiv=3)

	param = {
		"base_image": temp_img,      # The clean image with obstacles drawn
		"image": temp_img.copy(),    # Will get updated as we plot paths
		"obstacles": obstacles,
		"obstacle_waypoints": obstacle_waypoints,
		"outer_waypoints": outer_waypoints,

		"start": None,
		"destinations": [],          # We'll store multiple destinations
		"path_segments": [],         # Each segment from last point to next destination
	}

	cv2.namedWindow("Path Planning")
	cv2.setMouseCallback("Path Planning", click_event, param)

	print("Menu:\n"
		"  Left-click to set Start (first click), then multiple Destinations.\n"
		"  Press 'r' to reset path.\n"
		"  Press 'p' to replay the entire route.\n"
		"  Press 'q' or ESC to quit.\n")

	while True:
		cv2.imshow("Path Planning", param["image"])
		# Listen for keystrokes; handle menu
		quit_flag = handle_keyboard(param)
		if quit_flag:
			break

	cv2.destroyAllWindows()
