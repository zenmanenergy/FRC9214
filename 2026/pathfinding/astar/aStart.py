import cv2
import numpy as np
import os
import json
import heapq
import math

def draw_grid(image, square_size=10, color=(0, 255, 0), thickness=1):
	# Get image dimensions and prepare a list of grid positions.
	height, width = image.shape[:2]
	grid_positions = []
	# Draw the grid and store square positions.
	for x in range(0, width, square_size):
		for y in range(0, height, square_size):
			grid_positions.append(((x, y), (x + square_size, y + square_size)))
			cv2.rectangle(image, (x, y), (x + square_size, y + square_size), color, thickness)
	return image, grid_positions

def load_obstacles(file_path):
	if os.path.exists(file_path):
		with open(file_path, 'r') as file:
			return json.load(file)
	return {}

def heuristic(a, b, square_size):
	# Compute the octile distance for diagonal moves.
	# Convert pixel differences to number of cells.
	dx = abs(a[0] - b[0]) / square_size
	dy = abs(a[1] - b[1]) / square_size
	D = 1
	D2 = math.sqrt(2)
	return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

def astar(start, goal, grid_states, square_size, image):
	open_set = []
	heapq.heappush(open_set, (0, start))
	came_from = {}
	g_score = {start: 0}
	f_score = {start: heuristic(start, goal, square_size)}
	
	while open_set:
		_, current = heapq.heappop(open_set)
		if current == goal:
			path = []
			while current in came_from:
				path.append(current)
				current = came_from[current]
			return path[::-1]
		
		# Generate neighbors including diagonal moves.
		neighbors = [
			((current[0] + square_size, current[1]), 1),
			((current[0] - square_size, current[1]), 1),
			((current[0], current[1] + square_size), 1),
			((current[0], current[1] - square_size), 1),
			((current[0] + square_size, current[1] + square_size), math.sqrt(2)),
			((current[0] + square_size, current[1] - square_size), math.sqrt(2)),
			((current[0] - square_size, current[1] + square_size), math.sqrt(2)),
			((current[0] - square_size, current[1] - square_size), math.sqrt(2))
		]
		
		for neighbor, move_cost in neighbors:
			# Skip neighbor if it falls outside the image bounds.
			if neighbor[0] < 0 or neighbor[1] < 0 or neighbor[0] >= image.shape[1] or neighbor[1] >= image.shape[0]:
				continue
			# Skip neighbor if it is an obstacle.
			if f"{neighbor[0]},{neighbor[1]}" in grid_states and grid_states[f"{neighbor[0]},{neighbor[1]}"] == "occupied":
				continue
			
			temp_g_score = g_score[current] + move_cost
			if neighbor not in g_score or temp_g_score < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				h = heuristic(neighbor, goal, square_size)
				f = temp_g_score + h
				f_score[neighbor] = f
				heapq.heappush(open_set, (f, neighbor))
				# Add A* numbers (G, H, F) to the grid cell.
				# text = f"G:{temp_g_score:.1f} H:{h:.1f} F:{f:.1f}"
				# cv2.putText(image, text, (neighbor[0] + 5, neighbor[1] + 25), 
				# 	cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
	return []

def click_event(event, x, y, flags, param):
	if event == cv2.EVENT_LBUTTONDOWN:
		coord_key = f"{(x // param['square_size']) * param['square_size']},{(y // param['square_size']) * param['square_size']}"
		if param['click_count'] == 0:
			param['start'] = tuple(map(int, coord_key.split(',')))
			# Color the start cell in blue.
			cv2.rectangle(param['image'], param['start'], 
				(param['start'][0] + param['square_size'], param['start'][1] + param['square_size']), (255, 0, 0), -1)
			param['click_count'] += 1
		elif param['click_count'] == 1:
			param['goal'] = tuple(map(int, coord_key.split(',')))
			# Color the goal cell in yellow.
			cv2.rectangle(param['image'], param['goal'], 
				(param['goal'][0] + param['square_size'], param['goal'][1] + param['square_size']), (0, 255, 255), -1)
			param['click_count'] += 1
			path = astar(param['start'], param['goal'], param['grid_states'], param['square_size'], param['image'])
			# Highlight the computed shortest path in green.
			for p in path:
				cv2.rectangle(param['image'], (p[0], p[1]), 
					(p[0] + param['square_size'], p[1] + param['square_size']), (0, 255, 0), -1)
			cv2.imshow("Grid Overlay", param['image'])

if __name__ == "__main__":
	script_dir = os.path.dirname(os.path.abspath(__file__))
	image_path = os.path.join(script_dir, "REEFSCAPE2025.png")
	obstacle_file = os.path.join(script_dir, "obstacles.json")
	
	image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
	if image is None:
		raise ValueError("Image not found or unable to load.")
	
	square_size = 10
	image, grid_positions = draw_grid(image, square_size=square_size)
	grid_states = load_obstacles(obstacle_file)
	
	# Mark obstacles on the grid (fill with red).
	for coord_key, state in grid_states.items():
		if state == 'occupied':
			x1, y1 = map(int, coord_key.split(','))
			x2, y2 = x1 + square_size, y1 + square_size
			cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), -1)
	
	cv2.imshow("Grid Overlay", image)
	cv2.setMouseCallback("Grid Overlay", click_event, {
		'grid_positions': grid_positions,
		'image': image,
		'grid_states': grid_states,
		'square_size': square_size,
		'click_count': 0,
		'start': None,
		'goal': None
	})
	cv2.waitKey(0)
	cv2.destroyAllWindows()
