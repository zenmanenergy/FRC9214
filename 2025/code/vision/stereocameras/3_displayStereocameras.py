import os
import cv2
import numpy as np

# Load stereo calibration data
calibration_file = "stereo_calibration_data.npz"
current_dir = os.path.dirname(os.path.abspath(__file__))
calibration_path = os.path.join(current_dir, "calibration", calibration_file)

if not os.path.exists(calibration_path):
	print(f"Calibration file not found: {calibration_path}")
	exit()

# Load calibration data
calib_data = np.load(calibration_path)
mtxL = calib_data['mtxL']
distL = calib_data['distL']
mtxR = calib_data['mtxR']
distR = calib_data['distR']
R = calib_data['R']
T = calib_data['T']
R1 = calib_data['R1']
R2 = calib_data['R2']
P1 = calib_data['P1']
P2 = calib_data['P2']
Q = calib_data['Q']

print("Calibration data loaded successfully.")

# Open cameras
cap_left = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap_right = cv2.VideoCapture(1, cv2.CAP_DSHOW)

if not cap_left.isOpened() or not cap_right.isOpened():
	print("Error: Unable to open one or both cameras.")
	exit()

print("Cameras opened successfully.")

# Set camera resolution
cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Get camera resolution
width = int(cap_left.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap_left.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera resolution: {width}x{height}")

# Check calibration resolution
print(f"Calibration Matrix Dimensions: {mtxL.shape}")

# Set up undistortion and rectification maps
left_map1, left_map2 = cv2.initUndistortRectifyMap(
	mtxL, distL, R1, P1, (width, height), cv2.CV_16SC2
)
right_map1, right_map2 = cv2.initUndistortRectifyMap(
	mtxR, distR, R2, P2, (width, height), cv2.CV_16SC2
)

# Debugging rectification maps
print(f"Left Map 1 Shape: {left_map1.shape}, Type: {left_map1.dtype}")
print(f"Left Map 2 Shape: {left_map2.shape}, Type: {left_map2.dtype}")
print(f"Right Map 1 Shape: {right_map1.shape}, Type: {right_map1.dtype}")
print(f"Right Map 2 Shape: {right_map2.shape}, Type: {right_map2.dtype}")

# Function to draw horizontal lines
def draw_horizontal_lines(img, color=(0, 255, 0), thickness=1, num_lines=10):
	h, w = img.shape[:2]
	step = h // num_lines
	for i in range(0, h, step):
		cv2.line(img, (0, i), (w, i), color, thickness)
	return img

while True:
	# Capture frames
	ret_left, frame_left = cap_left.read()
	ret_right, frame_right = cap_right.read()

	if not ret_left or not ret_right:
		print("Error: Unable to read frames from cameras.")
		break

	# Display raw frames
	cv2.imshow("Left Camera - Raw", frame_left)
	cv2.imshow("Right Camera - Raw", frame_right)

	# Debug rectification
	rectified_left = cv2.remap(frame_left, left_map1, left_map2, interpolation=cv2.INTER_LINEAR)
	rectified_right = cv2.remap(frame_right, right_map1, right_map2, interpolation=cv2.INTER_LINEAR)

	# Check if rectified frames are valid
	print(f"Rectified Left Shape: {rectified_left.shape}, Type: {rectified_left.dtype}")
	print(f"Rectified Right Shape: {rectified_right.shape}, Type: {rectified_right.dtype}")

	# Draw lines on rectified frames
	rectified_left_lines = draw_horizontal_lines(rectified_left.copy())
	rectified_right_lines = draw_horizontal_lines(rectified_right.copy())

	cv2.imshow("Left Camera - Rectified", rectified_left_lines)
	cv2.imshow("Right Camera - Rectified", rectified_right_lines)

	# Exit on 'q'
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release resources
cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
