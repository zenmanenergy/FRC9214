# Camera 1
# This script checks which camera indices are available on the system.
# It uses the OpenCV library to attempt to open cameras on the first 10 indices (0-9).
# For each index, it tries to access the camera, and if successful, it prints that the camera is available.
# Otherwise, it prints that the camera is not available. This is useful for determining which
# cameras are recognized by OpenCV on a given system.

import cv2

print("\n" * 3)

for i in range(10):
	cap = cv2.VideoCapture(i)
	if cap.isOpened():
		print(f"Camera index {i} is available.")
		cap.release()
	else:
		print(f"Camera index {i} is not available.")
