# This script checks which camera indices are available on the system.
# It uses the OpenCV library to attempt to open cameras on the first 10 indices (0-9).
# For each index, it tries to access the camera, and if successful, it prints that the camera is available.
# Otherwise, it prints that the camera is not available. This is useful for determining which
# cameras are recognized by OpenCV on a given system.

import cv2

print("\n" * 3)  # Print three blank lines for spacing

# Loop through camera indices to check which are available
for i in range(10):  # Try checking first 10 indices
	# Try to open a camera at index i
	cap = cv2.VideoCapture(i)
	if cap.isOpened():
		# If the camera at this index can be accessed, print that it's available
		print(f"Camera index {i} is available.")
		cap.release()  # Release the camera resource
	else:
		# If the camera at this index can't be accessed, print that it's not available
		print(f"Camera index {i} is not available.")
