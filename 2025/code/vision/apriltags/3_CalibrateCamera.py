import cv2
import numpy as np
import glob
import os

# Get the current directory of this script
current_dir = os.path.dirname(os.path.abspath(__file__))
print(f"Current directory: {current_dir}")

# Define the size of the checkerboard (number of inner corners per chessboard row and column)
CHECKERBOARD = (9, 6)

# Set the size of each square on your checkerboard (in meters, e.g., 0.024 means each square is 2.4 cm)
square_size = 0.024

# Termination criteria for the iterative algorithm to refine corner detection
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on real-world measurements (a grid of points in 3D space)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all the images
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Load calibration images (assuming you have taken multiple checkerboard images)
images_path = os.path.join(current_dir, 'calibration', 'images', '*.png')
images = glob.glob(images_path)  # Replace with your folder path

# Check if any images are loaded
if len(images) == 0:
	print(f"No images found in {images_path}. Please check your folder and image paths.")
	exit()

print(f"Found {len(images)} calibration images.")

# Variable to store the image size
image_size = None

for image_file in images:
	print(f"Processing image: {image_file}")
	img = cv2.imread(image_file)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	# Find the chessboard corners
	ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

	# If found, refine the corner detection and add points
	if ret:
		objpoints.append(objp)
		corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
		imgpoints.append(corners2)

		# Set the image size after reading the first valid image
		if image_size is None:
			image_size = gray.shape[::-1]

		# Draw and display the corners
		cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
		cv2.imshow('Checkerboard', img)
		cv2.waitKey(500)
	else:
		print(f"Checkerboard corners not found in {image_file}")

cv2.destroyAllWindows()

# Ensure we have valid image size for calibration
if image_size is not None:
	# Perform camera calibration using the collected points
	print("Calibrating the camera...")
	ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

	# Check if calibration was successful
	if ret:
		# Apply manual offset to the principal point (c_x, c_y)
		manual_x_offset = 150  # 150 pixels as you discovered
		manual_y_offset = 0     # No vertical offset needed, adjust if necessary

		# Apply the offset to the principal point (c_x and c_y) in the camera matrix
		camera_matrix[0, 2] += manual_x_offset  # Adjust c_x
		camera_matrix[1, 2] += manual_y_offset  # Adjust c_y (if needed)

		# Save the camera matrix and distortion coefficients
		calibration_dir = os.path.join(current_dir, 'calibration')
		os.makedirs(calibration_dir, exist_ok=True)  # Ensure the directory exists

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
