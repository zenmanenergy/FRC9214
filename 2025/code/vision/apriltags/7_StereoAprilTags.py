import os
import cv2
import numpy as np
import math

# Baseline distance between the two cameras (in meters)
baseline_distance = 0.1  # For example, 10 cm apart

# Load calibration data for both cameras
current_dir = os.path.dirname(os.path.abspath(__file__))

# Load calibration data for camera 1
camera_matrix_1 = np.load(os.path.join(current_dir, 'calibration_camera1', 'camera_matrix.npy'))
dist_coeffs_1 = np.load(os.path.join(current_dir, 'calibration_camera1', 'dist_coeffs.npy'))

# Load calibration data for camera 2
camera_matrix_2 = np.load(os.path.join(current_dir, 'calibration_camera2', 'camera_matrix.npy'))
dist_coeffs_2 = np.load(os.path.join(current_dir, 'calibration_camera2', 'dist_coeffs.npy'))

# Define the AprilTag dictionary
tag_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# Define the size of the AprilTag in meters (15 cm = 0.15 meters)
tag_size = 0.15  # 15 cm in meters

# Initialize both cameras
cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)

while True:
	# Capture frames from both cameras
	ret1, frame1 = cap1.read()
	ret2, frame2 = cap2.read()

	if not ret1 or not ret2:
		break

	# Undistort both frames using their respective calibration data
	undistorted_frame1 = cv2.undistort(frame1, camera_matrix_1, dist_coeffs_1, None, camera_matrix_1)
	undistorted_frame2 = cv2.undistort(frame2, camera_matrix_2, dist_coeffs_2, None, camera_matrix_2)

	# Convert both frames to grayscale
	gray1 = cv2.cvtColor(undistorted_frame1, cv2.COLOR_BGR2GRAY)
	gray2 = cv2.cvtColor(undistorted_frame2, cv2.COLOR_BGR2GRAY)

	# Detect AprilTags in both camera feeds
	corners1, ids1, _ = cv2.aruco.detectMarkers(gray1, tag_dictionary)
	corners2, ids2, _ = cv2.aruco.detectMarkers(gray2, tag_dictionary)

	if ids1 is not None and ids2 is not None:
		# Assume the same tag is detected in both cameras (ensure ids match)
		if np.array_equal(ids1, ids2):
			# Estimate pose from both cameras
			retval1, rvecs1, tvecs1 = cv2.aruco.estimatePoseSingleMarkers(corners1, tag_size, camera_matrix_1, dist_coeffs_1)
			retval2, rvecs2, tvecs2 = cv2.aruco.estimatePoseSingleMarkers(corners2, tag_size, camera_matrix_2, dist_coeffs_2)

			if retval1 is not None and retval2 is not None:
				for rvec1, tvec1, rvec2, tvec2 in zip(rvecs1, tvecs1, rvecs2, tvecs2):
					# Reshape vectors to ensure they are in the correct shape (1x3 or 3x1)
					tvec1 = tvec1.reshape((3, 1))
					tvec2 = tvec2.reshape((3, 1))

					# Calculate midpoint between the two cameras
					midpoint = (tvec1 + tvec2) / 2

					# Draw detected markers and pose axes on both frames
					cv2.aruco.drawDetectedMarkers(undistorted_frame1, corners1)
					cv2.aruco.drawDetectedMarkers(undistorted_frame2, corners2)
					cv2.drawFrameAxes(undistorted_frame1, camera_matrix_1, dist_coeffs_1, rvec1, tvec1, 0.1)
					cv2.drawFrameAxes(undistorted_frame2, camera_matrix_2, dist_coeffs_2, rvec2, tvec2, 0.1)

					# Calculate the distance from the midpoint to the tag
					distance = np.linalg.norm(midpoint)
					print(f"Midpoint distance to tag: {distance:.2f} meters")

					# Calculate the average rotation vector (for yaw and pitch)
					rvec_avg = (rvec1 + rvec2) / 2
					rotation_matrix, _ = cv2.Rodrigues(rvec_avg)

					# Extract yaw and pitch from the rotation matrix
					pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
					yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

					# Convert radians to degrees
					pitch_deg = math.degrees(pitch)
					yaw_deg = math.degrees(yaw)

					print(f"Midpoint yaw: {yaw_deg:.2f} degrees, Midpoint pitch: {pitch_deg:.2f} degrees")

	# Display the frames from both cameras
	cv2.imshow('Camera 1 - AprilTag Detection', undistorted_frame1)
	cv2.imshow('Camera 2 - AprilTag Detection', undistorted_frame2)

	# Break the loop if 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release both cameras and close OpenCV windows
cap1.release()
cap2.release()
cv2.destroyAllWindows()
