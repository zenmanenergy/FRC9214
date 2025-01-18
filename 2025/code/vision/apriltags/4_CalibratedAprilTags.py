# This script captures video from a camera and detects AprilTags in real-time.
# It estimates the pose of each detected AprilTag using a pre-calibrated camera matrix
# and distortion coefficients, which allow for real-world distance and orientation measurements.
# The estimated pose is visualized by overlaying 3D axes on each detected AprilTag.

import cv2
import numpy as np
import os

# Load camera calibration parameters (replace with your actual calibrated values)
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(os.path.join(current_dir, "calibration", "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "calibration", "dist_coeffs.npy"))

# Define the real-world size of the AprilTag (in meters)
tag_size = 0.165  # Example: 16.5 cm

# Load the AprilTag dictionary (36h11 is a widely used standard dictionary)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# Initialize the camera and set its resolution
<<<<<<< HEAD
cap = cv2.VideoCapture(0)
=======
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
>>>>>>> dfd2eadb5679d3088169354329f397fb858c85e6
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
	# Capture each frame from the camera
	ret, frame = cap.read()
	if not ret:
		print("Failed to grab frame")
		break

	# Convert the frame to grayscale, as tag detection is typically done on grayscale images
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Detect AprilTags in the frame
	corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)

	if ids is not None:
		# Estimate the pose of each detected AprilTag
		rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, tag_size, camera_matrix, dist_coeffs)

		for rvec, tvec in zip(rvecs, tvecs):
			# Draw the detected markers on the frame
			cv2.aruco.drawDetectedMarkers(frame, corners, ids)

			# Draw the 3D axes to represent the pose (rotation and translation) of each tag
			cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)  # 0.1 is the length of each axis

	# Display the frame with detected tags and pose axes
	cv2.imshow('AprilTag Detection', frame)

	# Break the loop when 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
