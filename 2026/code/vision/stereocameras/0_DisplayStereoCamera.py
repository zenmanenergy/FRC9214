import cv2
import numpy as np

# Open cameras with DirectShow backend
cap_left = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Left camera
cap_right = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Right camera

# Set resolution
width = 1280
height = 720
cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

# StereoBM matcher for disparity map
stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)

while True:
	# Read frames
	ret_left, frame_left = cap_left.read()
	ret_right, frame_right = cap_right.read()

	if not ret_left or not ret_right:
		print("Error: Unable to fetch frames.")
		break

	# Convert frames to grayscale
	gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
	gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

	# Compute disparity map
	disparity = stereo.compute(gray_left, gray_right)

	# Normalize disparity map for visualization
	disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
	disparity_normalized = np.uint8(disparity_normalized)

	# Display images
	cv2.imshow('Left Camera', frame_left)
	cv2.imshow('Right Camera', frame_right)
	cv2.imshow('Disparity Map', disparity_normalized)

	# Exit on 'q'
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release resources
cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
