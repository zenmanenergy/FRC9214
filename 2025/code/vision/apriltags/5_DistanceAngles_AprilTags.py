# This script detects AprilTags in real-time using a calibrated camera, estimates their pose,
# and calculates the distance and angles to each detected tag.
# The calculated distance is adjusted by a correction factor to improve accuracy.
# The resulting information is displayed on the video feed, showing the tag ID, corrected distance,
# horizontal angle, and vertical angle for each detected tag.

import cv2
import numpy as np
import os

# Load the updated camera calibration data
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(os.path.join(current_dir, "calibration", "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "calibration", "dist_coeffs.npy"))

# Define the real-world size of the AprilTag (in meters)
tag_size = 0.164  # Example: 16.4 cm

# Load the AprilTag dictionary (36h11 is commonly used)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# Initialize the camera and set resolution
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Correction factor to improve distance measurement accuracy (based on calibration tests)
adjustment_factor = 30 / 45  # Adjusted using actual vs. measured distance

while True:
	# Capture each frame from the camera
	ret, frame = cap.read()
	if not ret:
		print("Failed to grab frame")
		break

	# Convert to grayscale (required for AprilTag detection)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Detect AprilTags in the frame
	corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)

	if ids is not None:
		# Estimate the pose (position and orientation) of each detected AprilTag
		rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, tag_size, camera_matrix, dist_coeffs)

		for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
			# Draw the detected markers on the frame
			cv2.aruco.drawDetectedMarkers(frame, corners, ids)

			# Draw axes on the tag to indicate orientation and position
			cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

			# Calculate the center of the tag (average of the four corners)
			tag_center_x = np.mean(corners[i][0][:, 0])
			tag_center_y = np.mean(corners[i][0][:, 1])

			# Calculate horizontal and vertical angles using the principal point in the camera matrix
			angle_x = np.arctan((tag_center_x - camera_matrix[0, 2]) / camera_matrix[0, 0])
			angle_y = np.arctan((tag_center_y - camera_matrix[1, 2]) / camera_matrix[1, 1])

			# Convert angles from radians to degrees for easier interpretation
			angle_x_deg = np.degrees(angle_x)
			angle_y_deg = np.degrees(angle_y)

			# Retrieve the Z translation value as the distance in meters
			distance = tvec[0][2]

			# Apply the adjustment factor to improve accuracy of the distance measurement
			corrected_distance = distance * adjustment_factor

			# Display the tag information (ID, corrected distance, angles) on the frame
			text = (f"ID: {ids[i][0]}, Distance: {corrected_distance:.2f}m, "
					f"Horizontal Angle: {angle_x_deg:.2f}°, Vertical Angle: {angle_y_deg:.2f}°")
			cv2.putText(frame, text, (10, 50 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

	# Display the frame with overlaid information
	cv2.imshow('AprilTag Detection', frame)

	# Exit the loop when 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release the camera and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()
