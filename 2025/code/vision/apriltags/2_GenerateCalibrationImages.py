import os
import cv2
import re

# Parameters
num_images_to_capture = 20
checkerboard_size = (9, 7)  # Inner corners per a chessboard row and column

current_dir = os.path.dirname(os.path.abspath(__file__))
images_dir = os.path.join(current_dir, 'calibration', 'images')
os.makedirs(images_dir, exist_ok=True)

# ✅ Find the highest existing calibration image index
existing_files = [f for f in os.listdir(images_dir) if f.startswith("calibration_image_") and f.endswith(".png")]
existing_indices = [int(re.search(r'\d+', f).group()) for f in existing_files if re.search(r'\d+', f)]
next_index = max(existing_indices, default=0) + 1  # Continue numbering from the last used index

# Open the camera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Adjust camera ID if necessary

# ✅ Dynamically set the highest supported resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)  # Change this if your camera supports higher (e.g., 3840 for 4K)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# ✅ Confirm actual resolution
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera Resolution: {actual_width}x{actual_height}")

captured_images = 0

def capture_image(event, x, y, flags, param):
	global captured_images, next_index
	if event == cv2.EVENT_LBUTTONDOWN and captured_images < num_images_to_capture:
		# Save the raw frame (unmodified) to disk without overwriting
		image_path = os.path.join(images_dir, f'calibration_image_{next_index}.png')
		cv2.imwrite(image_path, param['raw_frame'])
		print(f"Saved {image_path} at resolution {param['raw_frame'].shape[1]}x{param['raw_frame'].shape[0]}")
		captured_images += 1
		next_index += 1  # Increment file index

cv2.namedWindow('Calibration Image Capture')
cv2.setMouseCallback('Calibration Image Capture', capture_image)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

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
		corners = cv2.cornerSubPix(gray_frame, corners, (11, 11), (-1, -1), criteria)
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
