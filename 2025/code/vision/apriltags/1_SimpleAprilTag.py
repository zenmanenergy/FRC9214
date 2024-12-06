# Camera 3
#
# Highlight April tag
#
# This script captures video frames from a camera and detects AprilTags in real time.
# AprilTags are a type of fiducial marker commonly used for localization in robotics and
# computer vision applications. The script highlights detected tags in the video feed 
# using OpenCV's ArUco library. Note: Pose estimation is not performed as camera calibration
# parameters are not provided.

import cv2
import numpy as np

tag_size = 0.165  # Size of the AprilTag in meters (example: 16.5 cm)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
	ret, frame = cap.read()
	if not ret:
		print("Failed to grab frame")
		break

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)

	if ids is not None:
		cv2.aruco.drawDetectedMarkers(frame, corners, ids)

	cv2.imshow('AprilTag Detection', frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
