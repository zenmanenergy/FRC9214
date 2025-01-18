import cv2
import os

# Set camera ID
camera_id = 1  # Update based on your camera ID
cam = cv2.VideoCapture(camera_id)

# Set resolution (combined stereo image width and height)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)  # Combined width of left+right
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # Height of single image

# Create output folder
output_folder = "calibration_images"
os.makedirs(output_folder, exist_ok=True)

# Helper function to split the frame into left and right images
def split_frames(frame):
	height, width, _ = frame.shape
	midpoint = width // 2
	return frame[:, :midpoint], frame[:, midpoint:]

# Initialize variables
count = 0
prev_left = None
prev_right = None

while True:
	# Read frame
	ret, frame = cam.read()
	if not ret:
		print("Error: Unable to fetch frame.")
		break

	# Split into left and right
	left, right = split_frames(frame)

	# Check synchronization by comparing successive frames
	if prev_left is not None and prev_right is not None:
		diff_left = cv2.absdiff(prev_left, left)
		diff_right = cv2.absdiff(prev_right, right)

		# Synchronization threshold (adjust if necessary)
		sync_threshold = 10
		if (diff_left.mean() < sync_threshold) and (diff_right.mean() < sync_threshold):
			# Display synchronized images
			cv2.imshow("Left", left)
			cv2.imshow("Right", right)

			# Press 'c' to capture images
			key = cv2.waitKey(1) & 0xFF
			if key == ord('c'):
				cv2.imwrite(f"{output_folder}/left_{count}.png", left)
				cv2.imwrite(f"{output_folder}/right_{count}.png", right)
				print(f"Captured pair {count}")
				count += 1

	# Store previous frames
	prev_left, prev_right = left, right

	# Press 'q' to quit
	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		break

# Release resources
cam.release()
cv2.destroyAllWindows()
