import os
import cv2
import numpy as np
import glob

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Checkerboard dimensions (adjust as needed)
checkerboard_size = (9, 6)
square_size = 0.025  # Square size in meters (adjust if different)

# Prepare object points (3D coordinates)
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
objp *= square_size  # Scale based on square size

# Arrays to store object points and image points
objpoints = []  # 3D points in real world space
imgpoints_left = []  # 2D points in left image
imgpoints_right = []  # 2D points in right image

# Set up directories
current_dir = os.path.dirname(os.path.abspath(__file__))
images_dir = os.path.join(current_dir, 'calibration', 'images')
os.makedirs(images_dir, exist_ok=True)

# Load images
left_images = sorted(glob.glob('calibration_images/left_*.png'))
right_images = sorted(glob.glob('calibration_images/right_*.png'))

if len(left_images) != len(right_images):
	raise ValueError("Mismatch between the number of left and right images.")

if len(left_images) == 0:
	raise ValueError("No calibration images found in 'calibration_images' folder.")

for left_img_path, right_img_path in zip(left_images, right_images):
	# Read images
	imgL = cv2.imread(left_img_path)
	imgR = cv2.imread(right_img_path)

	if imgL is None or imgR is None:
		print(f"Error loading images: {left_img_path} or {right_img_path}")
		continue

	# Convert to grayscale
	grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
	grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

	# Find chessboard corners
	retL, cornersL = cv2.findChessboardCorners(grayL, checkerboard_size, None)
	retR, cornersR = cv2.findChessboardCorners(grayR, checkerboard_size, None)

	if retL and retR:
		# Refine corners
		cornersL = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
		cornersR = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)

		# Save points
		objpoints.append(objp)
		imgpoints_left.append(cornersL)
		imgpoints_right.append(cornersR)

# Calibrate left and right cameras
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, imgpoints_left, grayL.shape[::-1], None, None)
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, imgpoints_right, grayR.shape[::-1], None, None)

print(f"Left Camera Calibration RMS Error: {retL}")
print(f"Right Camera Calibration RMS Error: {retR}")

# Stereo calibration
retval, mtxL, distL, mtxR, distR, R, T, E, F = cv2.stereoCalibrate(
	objpoints, imgpoints_left, imgpoints_right,
	mtxL, distL, mtxR, distR, grayL.shape[::-1],
	criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC
)

print(f"Stereo Calibration RMS Error: {retval}")

# Stereo rectification
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
	mtxL, distL, mtxR, distR, grayL.shape[::-1], R, T
)

# Save calibration data
np.savez("stereo_calibration_data.npz", 
	mtxL=mtxL, distL=distL, mtxR=mtxR, distR=distR, R=R, T=T, R1=R1, R2=R2, P1=P1, P2=P2, Q=Q)

print("Calibration complete. Data saved to 'stereo_calibration_data.npz'")
