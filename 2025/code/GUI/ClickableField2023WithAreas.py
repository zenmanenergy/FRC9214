import tkinter as tk
from PIL import Image, ImageTk
import os
import json

# Define the field areas and initialize the coordinates dictionary
areas = ["red charging station", "red loading zone", "red community",
         "blue charging station", "blue loading zone", "blue community"]
coordinates = {area: [] for area in areas}
current_area = 0

# Get the path to the field.png image in the same directory as this script
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, "field.png")
json_path = os.path.join(script_dir, "field_coordinates.json")

# Initialize the tkinter window
root = tk.Tk()
root.title("FIRST Robotics 2023 Field")

# Load and display the image
image = Image.open(image_path)
photo = ImageTk.PhotoImage(image)

# Create a canvas to display the image
canvas = tk.Canvas(root, width=photo.width(), height=photo.height())
canvas.pack()
canvas.create_image(0, 0, anchor=tk.NW, image=photo)

# Check if coordinates file already exists
if os.path.exists(json_path):
	# Load coordinates from the JSON file
	with open(json_path, "r") as f:
		coordinates = json.load(f)
	label = tk.Label(root, text="Coordinates loaded from file. Areas drawn on field.")
	label.pack()
	# Draw the loaded areas
	def draw_existing_areas():
		for area, points in coordinates.items():
			if len(points) > 1:
				# Draw lines between points in each area
				for i in range(len(points)):
					x1, y1 = points[i]
					x2, y2 = points[(i + 1) % len(points)]  # Connect last point back to the first
					canvas.create_line(x1, y1, x2, y2, fill="blue", width=2)
				# Find the center of the area for label placement
				center_x = sum([p[0] for p in points]) // len(points)
				center_y = sum([p[1] for p in points]) // len(points)
				canvas.create_text(center_x, center_y, text=area, anchor=tk.CENTER, fill="blue", font=("Arial", 8, "bold"))
	# Draw the areas and labels from the loaded data
	draw_existing_areas()
else:
	label = tk.Label(root, text=f"Click to mark corners of {areas[current_area]}. Press Enter to save.")
	label.pack()

	# Function to handle clicking on the canvas to select corners
	def on_click(event):
		global coordinates, current_area
		# Add the clicked coordinates to the current area's list
		coordinates[areas[current_area]].append((event.x, event.y))
		
		# Display the selected point on the canvas
		canvas.create_text(event.x, event.y, text=f"({event.x}, {event.y})", anchor=tk.NW, fill="red", font=("Arial", 10))

	# Function to handle pressing Enter to save coordinates and move to the next area
	def on_enter(event):
		global current_area
		# Check if there are enough points for the current area
		if len(coordinates[areas[current_area]]) < 2:
			label.config(text="Please click at least two points before pressing Enter.")
			return
		
		# Move to the next area or finish if all areas are done
		current_area += 1
		if current_area < len(areas):
			label.config(text=f"Click to mark corners of {areas[current_area]}. Press Enter to save.")
		else:
			# Save coordinates to a JSON file
			with open(json_path, "w") as f:
				json.dump(coordinates, f, indent=4)
			label.config(text="All areas marked. Coordinates saved to field_coordinates.json.")
			# Draw areas and labels on the canvas
			draw_existing_areas()

	# Bind click and Enter events
	canvas.bind("<Button-1>", on_click)
	root.bind("<Return>", on_enter)

# Start the tkinter main loop
root.mainloop()
