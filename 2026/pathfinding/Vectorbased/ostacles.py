import cv2
import numpy as np
import os
import json
import math

# File names (adjust if necessary)
IMAGE_FILENAME = "REEFSCAPE2025.png"
OBSTACLES_FILE = "obstacles.json"

# Threshold distance (in pixels) to detect a click near the first point
CLOSE_THRESHOLD = 10

def load_obstacles(file_path):
	"""
	Load obstacles from a JSON file.
	Expected JSON format:
	{
		"obstacles": [
			[[x1, y1], [x2, y2], [x3, y3], ...],
			...
		]
	}
	"""
	if os.path.exists(file_path):
		with open(file_path, "r") as file:
			data = json.load(file)
			return data.get("obstacles", [])
	return []

def save_obstacles(file_path, obstacles):
	"""
	Save obstacles (list of polygons) to a JSON file.
	"""
	with open(file_path, "w") as file:
		json.dump({"obstacles": obstacles}, file, indent=4)

def draw_existing_obstacles(img, obstacles):
	"""
	Draw each polygon obstacle on the image.
	Polygons are drawn with red outlines.
	"""
	for poly in obstacles:
		pts = np.array(poly, np.int32)
		pts = pts.reshape((-1, 1, 2))
		cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=2)

def distance(p1, p2):
	"""
	Euclidean distance between two points.
	"""
	return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def mouse_callback(event, x, y, flags, state):
	"""
	Mouse callback function to handle drawing of a new polygon.
	- Left-click: add a point.
	  * If the click is near the first point of the current polygon, the polygon is closed,
	    drawn permanently, added to the obstacles list, and saved.
	- Mouse move: (optional) show a preview line from the last point to the current mouse position.
	"""
	if event == cv2.EVENT_LBUTTONDOWN:
		pt = (x, y)
		# If this is the first point in the current polygon, add it.
		if len(state["current_polygon"]) == 0:
			state["current_polygon"].append(pt)
			# Draw a small circle to mark the starting point.
			cv2.circle(state["img_display"], pt, 3, (255, 0, 0), -1)
		else:
			# If the new point is near the first point, close the polygon.
			if distance(pt, state["current_polygon"][0]) < CLOSE_THRESHOLD:
				# Close the polygon by drawing a line from the last point to the first point.
				cv2.line(state["img_display"], state["current_polygon"][-1], state["current_polygon"][0], (255, 0, 0), 2)
				# Add the completed polygon to obstacles.
				state["obstacles"].append(state["current_polygon"][:])
				# Redraw the obstacle permanently (in red).
				pts = np.array(state["current_polygon"], np.int32)
				pts = pts.reshape((-1, 1, 2))
				cv2.polylines(state["img_display"], [pts], isClosed=True, color=(0, 0, 255), thickness=2)
				# Save updated obstacles to file.
				save_obstacles(state["obstacles_file"], state["obstacles"])
				print("Polygon saved:", state["current_polygon"])
				# Clear current polygon for a new drawing.
				state["current_polygon"] = []
			else:
				# Otherwise, add the new point and draw a line from the previous point.
				cv2.line(state["img_display"], state["current_polygon"][-1], pt, (255, 0, 0), 2)
				state["current_polygon"].append(pt)
		cv2.imshow("Obstacle Editor", state["img_display"])
	elif event == cv2.EVENT_MOUSEMOVE:
		# Optional: show a preview line from the last point to the current mouse position.
		temp_img = state["img_display"].copy()
		if len(state["current_polygon"]) > 0:
			cv2.line(temp_img, state["current_polygon"][-1], (x, y), (255, 0, 0), 1)
		cv2.imshow("Obstacle Editor", temp_img)

def main():
	# Get the directory of the current script.
	script_dir = os.path.dirname(os.path.abspath(__file__))
	
	# Build the full path for the image and obstacles file.
	image_path = os.path.join(script_dir, IMAGE_FILENAME)
	obstacles_file = os.path.join(script_dir, OBSTACLES_FILE)
	
	# Load the base image.
	img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
	if img is None:
		raise ValueError("Image not found or unable to load.")
	
	# Load existing obstacles.
	obstacles = load_obstacles(obstacles_file)
	# Create a copy of the image to draw on.
	img_display = img.copy()
	# Draw the existing obstacles on the image.
	draw_existing_obstacles(img_display, obstacles)
	
	# State dictionary to pass to the mouse callback.
	state = {
		"img_display": img_display,
		"obstacles": obstacles,           # list of polygons already loaded
		"current_polygon": [],            # list of points for the polygon being drawn
		"obstacles_file": obstacles_file  # file path for saving obstacles
	}
	
	cv2.namedWindow("Obstacle Editor")
	cv2.imshow("Obstacle Editor", img_display)
	cv2.setMouseCallback("Obstacle Editor", mouse_callback, state)
	
	print("Left-click to add points. Click near the first point to close the polygon.")
	print("Press 'q' to quit.")
	
	while True:
		key = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			break
	
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()
