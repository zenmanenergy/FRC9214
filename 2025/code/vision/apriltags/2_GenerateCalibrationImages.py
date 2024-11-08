# This script captures a specified number of images for camera calibration.
# It uses OpenCV to access the camera feed, displays instructions for the user, 
# and allows image capture with a left mouse click. Each captured image is saved
# in a designated "calibration/images" folder within the directory of the script.
# The script terminates automatically when the required number of images has been captured.

import os
import cv2

# Number of images to capture for calibration
num_images_to_capture = 20

# Get the current directory of the .py file
current_dir = os.path.dirname(os.path.abspath(__file__))

# Create the calibration images directory if it doesn't exist
images_dir = os.path.join(current_dir, 'calibration', 'images')
os.makedirs(images_dir, exist_ok=True)

# Initialize the camera
cap = cv2.VideoCapture(1)

# Set the camera resolution to 1920x1080
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Variable to store the count of captured images
captured_images = 0

# Mouse callback function to capture image on left mouse click
def capture_image(event, x, y, flags, param):
	global captured_images
	if event == cv2.EVENT_LBUTTONDOWN and captured_images < num_images_to_capture:
		# Save the captured image to the calibration images folder
		image_path = os.path.join(images_dir, f'calibration_image_{captured_images + 1}.png')
		cv2.imwrite(image_path, param)  # param holds the frame to be saved
		print(f"Saved {image_path}")

		# Increment the counter of captured images
		captured_images += 1

# Create a window for displaying the camera feed and set the mouse callback
cv2.namedWindow('Calibration Image Capture')
cv2.setMouseCallback('Calibration Image Capture', capture_image)

while captured_images < num_images_to_capture:
	# Capture each frame from the camera feed
	ret, frame = cap.read()
	if not ret:
		break  # Exit if no frame is captured

	# Display instructions and the count of captured images on the frame
	instructions = "Click to capture an image."
	cv2.putText(frame, instructions, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

	count_text = f"{captured_images}/{num_images_to_capture} captures"
	cv2.putText(frame, count_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

	# Show the camera feed with instructions and capture count
	cv2.imshow('Calibration Image Capture', frame)

	# Pass the current frame to the mouse callback function
	cv2.setMouseCallback('Calibration Image Capture', capture_image, frame)

	# Break the loop if 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

print("Calibration image capture complete.")
