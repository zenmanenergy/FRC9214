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

# Create custom detector parameters for improved accuracy
parameters = cv2.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 23
parameters.adaptiveThreshWinSizeStep = 10

# Initialize the camera and set its resolution
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
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

	# Undistort the image to minimize distortion-related inaccuracies
	undistorted = cv2.undistort(gray, camera_matrix, dist_coeffs)

	# Detect AprilTags in the undistorted frame
	corners, ids, rejected = cv2.aruco.detectMarkers(undistorted, dictionary, parameters=parameters)

	if ids is not None:
		# Estimate the pose of each detected AprilTag
		rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, tag_size, camera_matrix, dist_coeffs)

		for rvec, tvec, corner in zip(rvecs, tvecs, corners):
			# Draw the detected markers on the frame
			cv2.aruco.drawDetectedMarkers(frame, corners, ids)

			# Refine corner positions to subpixel accuracy
			cv2.cornerSubPix(
				gray, corner, (5, 5), (-1, -1),
				criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
				
			)

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
