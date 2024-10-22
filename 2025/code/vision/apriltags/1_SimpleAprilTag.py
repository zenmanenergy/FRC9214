import cv2
import numpy as np

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
		# Since calibration is removed, we won't estimate the pose, but we can still detect and highlight the tags
		cv2.aruco.drawDetectedMarkers(frame, corners, ids)

	# Display the resulting frame
	cv2.imshow('AprilTag Detection', frame)

	# Break the loop when 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
