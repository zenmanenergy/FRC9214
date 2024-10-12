import os
import cv2
import numpy as np
import random

# Number of images to capture for calibration (adjust as needed)
num_images_to_capture = 10

# Define the size of the checkerboard (inner corners per row and column)
CHECKERBOARD = (9, 6)

# Prepare object points based on the checkerboard size and square size
square_size = 0.025  # e.g., 2.5 cm per square (adjust based on your checkerboard)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Define the folder for camera 1 and camera 2 calibration images
calibration_folder1 = os.path.join(current_dir, 'calibration1')
calibration_folder2 = os.path.join(current_dir, 'calibration2')

# Create directories if they don't exist
os.makedirs(calibration_folder1, exist_ok=True)
os.makedirs(calibration_folder2, exist_ok=True)

# Function to generate a random guide box within the frame with variable sizes
def generate_random_guide_box(frame_width, frame_height):
	# Random rectangular box size between 100x150 and 300x400 pixels (width x height)
	box_width = random.randint(100, 300)
	box_height = random.randint(150, 400)
	x1 = random.randint(0, frame_width - box_width)
	y1 = random.randint(0, frame_height - box_height)
	x2 = x1 + box_width
	y2 = y1 + box_height
	return (x1, y1, x2, y2)

# Function to capture calibration images from a single camera
def capture_calibration_images(camera_id, folder, num_images):
	cap = cv2.VideoCapture(camera_id)
	captured_images = 0

	while captured_images < num_images:
		ret, frame = cap.read()
		if not ret:
			break

		frame_height, frame_width = frame.shape[:2]

		# Generate a random guide box size and position for this capture
		current_box = generate_random_guide_box(frame_width, frame_height)
		x1, y1, x2, y2 = current_box
		cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

		# Display instructions and count on the screen
		instructions = "Hold the checkerboard inside the green box."
		cv2.putText(frame, instructions, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

		count_text = f"{captured_images}/{num_images} captures"
		cv2.putText(frame, count_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

		# Convert the frame to grayscale
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		# Try to find the checkerboard corners in the frame
		ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

		# If checkerboard is found within the guide box, refine and save the image
		if ret:
			# Refine the corner detection
			corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), None)

			# Check if the checkerboard is within the guide box
			center_x, center_y = np.mean(corners2, axis=0).ravel()
			if x1 < center_x < x2 and y1 < center_y < y2:
				# Save the image to the calibration images folder
				image_path = os.path.join(folder, f'calibration_image_{captured_images + 1}.png')
				cv2.imwrite(image_path, frame)
				print(f"Saved {image_path}")

				captured_images += 1  # Increment the counter

				# Wait a moment before capturing the next image
				cv2.waitKey(500)

		# Show the camera feed with the guide box, instructions, and count
		cv2.imshow(f'Camera {camera_id} Calibration', frame)

		# Break the loop if 'q' is pressed
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()

# Capture images from camera 1
print("Capturing images from camera 1...")
capture_calibration_images(0, calibration_folder1, num_images_to_capture)

# Capture images from camera 2
print("Capturing images from camera 2...")
capture_calibration_images(1, calibration_folder2, num_images_to_capture)

print("Calibration image capture complete.")
