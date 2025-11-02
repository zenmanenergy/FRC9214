import tkinter as tk
from PIL import Image, ImageTk
import os

# Get the path to the field.png image in the same directory as this script
script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, "field.png")

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

# Function to display coordinates on click
def display_coordinates(event):
	# Display coordinates on the console
	print(f"Clicked at coordinates: ({event.x}, {event.y})")
	
	# Display coordinates on the canvas
	canvas.create_text(event.x, event.y, text=f"({event.x}, {event.y})", anchor=tk.NW, fill="red", font=("Arial", 10))

# Bind the click event to the function
canvas.bind("<Button-1>", display_coordinates)

# Start the tkinter main loop
root.mainloop()
