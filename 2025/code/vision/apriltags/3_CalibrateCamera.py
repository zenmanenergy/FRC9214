# This script performs camera calibration using a set of checkerboard images.
# It identifies checkerboard corners in the images, refines the detected corners,
# and calculates the camera matrix and distortion coefficients based on real-world
# measurements. After calibration, it applies a manual offset to the camera's principal
# point and saves the camera matrix and distortion coefficients to files.

import cv2
import numpy as np
import glob
import os

# Get the current directory of this script
current_dir = os.path.dirname(os.path.abspath(__file__))
print(f"Current directory: {current_dir}")

# Define the checkerboard dimensions (number of inner corners per row and column)
CHECKERBOARD = (9, 6)

# Define the real-world size of each square on the checkerboard (e.g., 0.024 meters means each square is 2.4 cm)
square_size = 0.024

# Termination criteria for refining corner detection accuracy
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on checkerboard size (3D points in real space)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store 3D object points and 2D image points from all images
objpoints = []  # Real-world 3D points
imgpoints = []  # 2D points in image plane

# Load calibration images from the specified directory
images_path = os.path.join(current_dir, 'calibration', 'images', '*.png')
images = glob.glob(images_path)

# Check if any images were loaded
if len(images) == 0:
	print(f"No images found in {images_path}. Please check your folder and image paths.")
	exit()

print(f"Found {len(images)} calibration images.")

# Variable to store the image size (used for calibration)
image_size = None

# Process each calibration image
for image_file in images:
	print(f"Processing image: {image_file}")
	img = cv2.imread(image_file)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	# Attempt to find the checkerboard corners
	ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

	# If corners are found, refine detection and add to the list of points
	if ret:
		objpoints.append(objp)
		corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
		imgpoints.append(corners2)

		# Set the image size based on the first valid image
		if image_size is None:
			image_size = gray.shape[::-1]

		# Draw detected corners on the image for visualization
		cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
		cv2.imshow('Checkerboard', img)
		cv2.waitKey(500)
	else:
		print(f"Checkerboard corners not found in {image_file}")

# Close the OpenCV windows
cv2.destroyAllWindows()

# Ensure a valid image size is set before proceeding
if image_size is not None:
	# Perform camera calibration with the collected points
	print("Calibrating the camera...")
	ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

	# Check if calibration was successful
	if ret:
		# Apply a manual offset to the principal point in the camera matrix
		manual_x_offset = 150  # Offset in pixels for x-axis
		manual_y_offset = 0    # Offset in pixels for y-axis (adjust if necessary)

		# Adjust principal point (c_x and c_y) in the camera matrix
		camera_matrix[0, 2] += manual_x_offset  # Adjust c_x
		camera_matrix[1, 2] += manual_y_offset  # Adjust c_y if needed

		# Save camera matrix and distortion coefficients to the calibration directory
		calibration_dir = os.path.join(current_dir, 'calibration')
		os.makedirs(calibration_dir, exist_ok=True)

		np.save(os.path.join(calibration_dir, "camera_matrix.npy"), camera_matrix)
		np.save(os.path.join(calibration_dir, "dist_coeffs.npy"), dist_coeffs)

		print("Calibration successful with manual offsets!")
		print("Camera matrix (with offset):")
		print(camera_matrix)
		print("Distortion coefficients:")
		print(dist_coeffs)
	else:
		print("Camera calibration failed. Please check your images and try again.")
else:
	print("No valid images found for calibration. Please check your images.")
