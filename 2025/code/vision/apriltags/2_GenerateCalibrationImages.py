# Camera 4: Calibrate Camera
# This script captures a specified number of images for camera calibration.
# It uses OpenCV to access the camera feed, displays instructions for the user, 
# and allows image capture with a left mouse click. Each captured image is saved
# in a designated "calibration/images" folder within the directory of the script.
# The script terminates automatically when the required number of images has been captured.

import os
import cv2

num_images_to_capture = 20

current_dir = os.path.dirname(os.path.abspath(__file__))

images_dir = os.path.join(current_dir, 'calibration', 'images')
os.makedirs(images_dir, exist_ok=True)

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

captured_images = 0

def capture_image(event, x, y, flags, param):
	global captured_images
	if event == cv2.EVENT_LBUTTONDOWN and captured_images < num_images_to_capture:
		image_path = os.path.join(images_dir, f'calibration_image_{captured_images + 1}.png')
		cv2.imwrite(image_path, param)
		print(f"Saved {image_path}")
		captured_images += 1

cv2.namedWindow('Calibration Image Capture')
cv2.setMouseCallback('Calibration Image Capture', capture_image)

while captured_images < num_images_to_capture:
	ret, frame = cap.read()
	if not ret:
		break

	instructions = "Click to capture an image."
	cv2.putText(frame, instructions, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

	count_text = f"{captured_images}/{num_images_to_capture} captures"
	cv2.putText(frame, count_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

	cv2.imshow('Calibration Image Capture', frame)

	cv2.setMouseCallback('Calibration Image Capture', capture_image, frame)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

print("Calibration image capture complete.")
