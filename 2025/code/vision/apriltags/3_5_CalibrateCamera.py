import cv2
import numpy as np
import glob
import os

# Get the current directory of this script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Define the checkerboard dimensions and square size
CHECKERBOARD = (9, 6)
square_size = 0.024

# Termination criteria for refining corner detection accuracy
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on checkerboard size (3D points in real space)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store 3D object points and 2D image points
objpoints = []
imgpoints = []

# Load calibration images
images_path = os.path.join(current_dir, 'calibration', 'images', '*.png')
images = glob.glob(images_path)

if len(images) == 0:
	print(f"No images found in {images_path}. Please check your folder and image paths.")
	exit()

print(f"Found {len(images)} calibration images.")

image_size = None

for image_file in images:
	img = cv2.imread(image_file)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

	if ret:
		objpoints.append(objp)
		corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
		imgpoints.append(corners2)

		if image_size is None:
			image_size = gray.shape[::-1]

		cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
		cv2.imshow('Checkerboard', img)
		cv2.waitKey(500)
	else:
		print(f"Checkerboard corners not found in {image_file}")

cv2.destroyAllWindows()

if image_size is not None:
	ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

	if ret:
		# Save calibration results
		calibration_dir = os.path.join(current_dir, 'calibration')
		os.makedirs(calibration_dir, exist_ok=True)

		np.save(os.path.join(calibration_dir, "camera_matrix.npy"), camera_matrix)
		np.save(os.path.join(calibration_dir, "dist_coeffs.npy"), dist_coeffs)

		# Compute reprojection error
		mean_error = 0
		for i in range(len(objpoints)):
			imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
			error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
			mean_error += error
		print(f"Calibration successful! Mean reprojection error: {mean_error / len(objpoints)}")
	else:
		print("Camera calibration failed. Please check your images.")
else:
	print("No valid images found for calibration.")
