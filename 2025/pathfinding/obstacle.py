import cv2
import numpy as np
import os

def draw_grid(image, square_size=50, color=(0, 255, 0), thickness=2):
	height, width = image.shape[:2]
	grid_positions = []

	# Draw grid and store square positions
	for x in range(0, width, square_size):
		for y in range(0, height, square_size):
			grid_positions.append(((x, y), (x + square_size, y + square_size)))
			cv2.rectangle(image, (x, y), (x + square_size, y + square_size), color, thickness)

	return image, grid_positions


def click_event(event, x, y, flags, param):
	if event == cv2.EVENT_LBUTTONDOWN:
		for idx, ((x1, y1), (x2, y2)) in enumerate(param['grid_positions']):
			if x1 <= x <= x2 and y1 <= y <= y2:
				if param['grid_states'][idx] == 'red':
					cv2.rectangle(param['image'], (x1, y1), (x2, y2), param['original_color'], -1)
					param['grid_states'][idx] = 'transparent'
				else:
					cv2.rectangle(param['image'], (x1, y1), (x2, y2), (0, 0, 255), -1)
					param['grid_states'][idx] = 'red'
				cv2.imshow("Grid Overlay", param['image'])

if __name__ == "__main__":
	# Set image path to be in the same directory as the script
	script_dir = os.path.dirname(os.path.abspath(__file__))
	image_path = os.path.join(script_dir, "REEFSCAPE2025.png")
	
	image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
	if image is None:
		raise ValueError("Image not found or unable to load.")
	
	square_size = 50
	original_color = (255, 255, 255, 0) if image.shape[-1] == 4 else (255, 255, 255)
	image, grid_positions = draw_grid(image, square_size=square_size)
	
	# Initialize grid states
	grid_states = ['transparent'] * len(grid_positions)

	cv2.imshow("Grid Overlay", image)
	cv2.setMouseCallback("Grid Overlay", click_event, {
		'grid_positions': grid_positions,
		'image': image,
		'grid_states': grid_states,
		'original_color': original_color
	})
	cv2.waitKey(0)
	cv2.destroyAllWindows()
