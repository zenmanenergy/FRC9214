# This script captures video frames from a camera and detects AprilTags in real time.
# AprilTags are a type of fiducial marker similar to QR codes, which are commonly used
# for localization in robotics and computer vision applications.
# This script uses OpenCV's ArUco library to identify and display AprilTags, highlighting
# detected tags on the video feed. Note: The script does not estimate tag poses, as camera
# calibration details are not provided.

import cv2
import numpy as np

# Define the size of the AprilTag in real-world units (meters)
tag_size = 0.165  # Example: 16.5 cm

# Load the AprilTag dictionary (36h11 is commonly used in robotics)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# Initialize the camera with DirectShow backend for better compatibility (especially on Windows)
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

# Set camera resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
	# Capture each frame from the video feed
	ret, frame = cap.read()
	if not ret:
		# If frame capture fails, print an error and exit the loop
		print("Failed to grab frame")
		break

	# Convert frame to grayscale for tag detection (AprilTag detection requires grayscale images)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Detect AprilTags in the frame
	corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)

	if ids is not None:
		# Draw the detected markers on the frame if any tags are identified
		cv2.aruco.drawDetectedMarkers(frame, corners, ids)

	# Display the frame with detected tags highlighted
	cv2.imshow('AprilTag Detection', frame)

	# Break the loop when 'q' key is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release the camera and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()
