import os
import cv2

# Number of images to capture for calibration (adjust as needed)
num_images_to_capture = 20

# Get the current directory of the .py file
current_dir = os.path.dirname(os.path.abspath(__file__))

# Create the calibration images directory if it doesn't exist
images_dir = os.path.join(current_dir, 'calibration', 'images')
os.makedirs(images_dir, exist_ok=True)

# Initialize the camera
cap = cv2.VideoCapture(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Variable to store the captured images count
captured_images = 0

# Mouse callback function to capture image on left mouse click
def capture_image(event, x, y, flags, param):
	global captured_images
	if event == cv2.EVENT_LBUTTONDOWN and captured_images < num_images_to_capture:
		# Save the image to the calibration images folder
		image_path = os.path.join(images_dir, f'calibration_image_{captured_images + 1}.png')
		cv2.imwrite(image_path, param)
		print(f"Saved {image_path}")

		captured_images += 1  # Increment the counter

# Create a window and set the mouse callback function to capture the image
cv2.namedWindow('Calibration Image Capture')
cv2.setMouseCallback('Calibration Image Capture', capture_image)

while captured_images < num_images_to_capture:
	ret, frame = cap.read()
	if not ret:
		break

	# Show instructions and count on the screen
	instructions = "Click to capture an image."
	cv2.putText(frame, instructions, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

	count_text = f"{captured_images}/{num_images_to_capture} captures"
	cv2.putText(frame, count_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

	# Show the camera feed
	cv2.imshow('Calibration Image Capture', frame)

	# Pass the current frame to the mouse callback function
	cv2.setMouseCallback('Calibration Image Capture', capture_image, frame)

	# Break the loop if 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release the camera and close OpenCV windows
cap.release()
cv2.destroyAllWindows()

print("Calibration image capture complete.")
