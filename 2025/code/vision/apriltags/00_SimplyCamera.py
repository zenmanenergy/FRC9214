# Camera 2
# Display an image from a camera
#
# This script captures video from a camera using OpenCV. It sets the resolution to 1920x1080
# and displays the video feed in a window titled "Verify camera." The loop continues until
# the 'q' key is pressed. It is useful for verifying camera functionality and resolution settings.

import cv2

cap0 = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap2 = cv2.VideoCapture(1, cv2.CAP_DSHOW)

cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
	ret2, frame2 = cap2.read()
	if not ret2:
		print("Failed to grab frame 2")
		break

	ret0, frame0 = cap0.read()
	if not ret0:
		print("Failed to grab frame 0")
		break

	cv2.imshow('Camera 0', frame0)
	cv2.imshow('Camera 2', frame2)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap0.release()
cap2.release()
cv2.destroyAllWindows()
