# Camera 2
# Display an image from a camera
#
# This script captures video from a camera using OpenCV. It sets the resolution to 1920x1080
# and displays the video feed in a window titled "Verify camera." The loop continues until
# the 'q' key is pressed. It is useful for verifying camera functionality and resolution settings.

import cv2

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
	ret, frame = cap.read()
	if not ret:
		print("Failed to grab frame")
		break

	cv2.imshow('Verify camera', frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
