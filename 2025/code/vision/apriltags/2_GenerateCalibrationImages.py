import os
import cv2

# Parameters
num_images_to_capture = 20
checkerboard_size = (9, 6)  # Inner corners per a chessboard row and column

current_dir = os.path.dirname(os.path.abspath(__file__))
images_dir = os.path.join(current_dir, 'calibration', 'images')
os.makedirs(images_dir, exist_ok=True)

# Open the camera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Adjust camera ID if necessary
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

captured_images = 0

def capture_image(event, x, y, flags, param):
	global captured_images
	if event == cv2.EVENT_LBUTTONDOWN and captured_images < num_images_to_capture:
		# Save the raw frame (unmodified) to disk
		image_path = os.path.join(images_dir, f'calibration_image_{captured_images + 1}.png')
		cv2.imwrite(image_path, param['raw_frame'])
		print(f"Saved {image_path}")
		captured_images += 1

cv2.namedWindow('Calibration Image Capture')
cv2.setMouseCallback('Calibration Image Capture', capture_image)

while captured_images < num_images_to_capture:
	ret, frame = cap.read()
	if not ret:
		break

	# Make a copy of the raw frame for saving
	raw_frame = frame.copy()

	# Find checkerboard corners for visual feedback
	gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	ret_corners, corners = cv2.findChessboardCorners(gray_frame, checkerboard_size, None)
	if ret_corners:
		cv2.drawChessboardCorners(frame, checkerboard_size, corners, ret_corners)

	# Display instructions and capture count
	instructions = "Click to capture an image."
	cv2.putText(frame, instructions, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

	count_text = f"{captured_images}/{num_images_to_capture} captures"
	cv2.putText(frame, count_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

	# Show the processed frame in the window
	cv2.imshow('Calibration Image Capture', frame)
	cv2.setMouseCallback('Calibration Image Capture', capture_image, {'raw_frame': raw_frame})

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

print("Calibration image capture complete.")
