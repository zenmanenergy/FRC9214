import cv2
import numpy as np
import os

# Camera parameters (example values, replace with calibrated values)
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(current_dir + "\\calibration\\" + "camera_matrix.npy")
dist_coeffs = np.load(current_dir + "\\calibration\\" + "dist_coeffs.npy")

# Define the size of the AprilTag (real-world size in meters)
tag_size = 0.165  # Example: 16.5 cm

# Load the AprilTag dictionary (36h11 is a common one)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# Initialize the camera
cap = cv2.VideoCapture(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


while True:
	ret, frame = cap.read()
	if not ret:
		print("Failed to grab frame")
		break

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Detect AprilTags
	corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)

	if ids is not None:
		# Estimate pose of the AprilTag
		rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, tag_size, camera_matrix, dist_coeffs)

		for rvec, tvec in zip(rvecs, tvecs):
			# Draw the detected markers
			cv2.aruco.drawDetectedMarkers(frame, corners, ids)

			# Use cv2.drawFrameAxes (ensure matrices are float32)
			cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)


	# Display the resulting frame
	cv2.imshow('AprilTag Detection', frame)

	# Break the loop when 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
