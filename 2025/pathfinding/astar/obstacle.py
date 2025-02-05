import cv2
import numpy as np
import os
import json

def draw_grid(image, square_size=10, color=(0, 255, 0), thickness=1):
	height, width = image.shape[:2]
	grid_positions = []

	# Draw grid and store square positions
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

def save_obstacles(file_path, obstacles):
	with open(file_path, 'w') as file:
		json.dump(obstacles, file, indent=4)

def click_event(event, x, y, flags, param):
	if event == cv2.EVENT_LBUTTONDOWN:
		for idx, ((x1, y1), (x2, y2)) in enumerate(param['grid_positions']):
			if x1 <= x <= x2 and y1 <= y <= y2:
				coord_key = f"{x1},{y1}"
				if param['grid_states'].get(coord_key, "available") == 'occupied':
					cv2.rectangle(param['image'], (x1, y1), (x2, y2), param['original_color'], -1)
					param['grid_states'][coord_key] = 'available'
				else:
					cv2.rectangle(param['image'], (x1, y1), (x2, y2), (0, 0, 255), -1)
					param['grid_states'][coord_key] = 'occupied'
				save_obstacles(param['obstacle_file'], param['grid_states'])
				cv2.imshow("Grid Overlay", param['image'])

if __name__ == "__main__":
	# Set image path to be in the same directory as the script
	script_dir = os.path.dirname(os.path.abspath(__file__))
	image_path = os.path.join(script_dir, "REEFSCAPE2025.png")
	obstacle_file = os.path.join(script_dir, "obstacles.json")
	
	image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
	if image is None:
		raise ValueError("Image not found or unable to load.")
	
	square_size = 10
	original_color = (255, 255, 255, 0) if image.shape[-1] == 4 else (255, 255, 255)
	image, grid_positions = draw_grid(image, square_size=square_size)
	
	# Load obstacle states
	grid_states = load_obstacles(obstacle_file)

	# Highlight occupied squares
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
		'original_color': original_color,
		'obstacle_file': obstacle_file
	})
	cv2.waitKey(0)
	cv2.destroyAllWindows()
