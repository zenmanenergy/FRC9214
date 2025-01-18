import cv2
import numpy as np
import os

# Load the updated camera calibration data
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(os.path.join(current_dir, "calibration", "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "calibration", "dist_coeffs.npy"))


print(camera_matrix)

# Define the real-world size of the AprilTag (in meters)
tag_size = 0.164  # Example: 16.4 cm

# Load the AprilTag dictionary (36h11 is commonly used)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# Initialize the camera and set resolution
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

<<<<<<< HEAD
# Correction factor to improve distance measurement accuracy (based on calibration tests)
adjustment_factor = 1  # Adjusted using actual vs. measured distance
=======
# Precompute distortion maps for undistortion
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, (frame_width, frame_height), cv2.CV_16SC2)

# Define correction parameters
base_adjustment_factor = 100 / 124  # Base adjustment factor for center
k_dynamic = 0.4  # Dynamic adjustment factor based on angle
k_proximity = -0.35  # Proximity scaling constant
reference_distance = 1.0  # Reference distance in meters (e.g., 100 cm)
max_radius = np.sqrt((frame_width / 2)**2 + (frame_height / 2)**2)

# Define 3D world coordinates for the tag corners
half_size = tag_size / 2
tag_corners_3d = np.array([
	[-half_size, -half_size, 0],
	[ half_size, -half_size, 0],
	[ half_size,  half_size, 0],
	[-half_size,  half_size, 0]
], dtype=np.float32)

# Function to calculate dynamic adjustment factor based on radial distance
def calculate_adjustment_factor(raw_distance, radial_distance, max_radius, base_factor, reference_distance, k_proximity):
	# Quadratic proximity scaling applied to the adjustment factor
	adjustment_factor = base_factor + k_dynamic * (radial_distance / max_radius)
	adjusted_distance = raw_distance * adjustment_factor
	adjustment_factor *= (1 + k_proximity * abs(reference_distance - adjusted_distance))
	return adjustment_factor
>>>>>>> dfd2eadb5679d3088169354329f397fb858c85e6

while True:
	# Capture each frame
	ret, frame = cap.read()
	if not ret:
		print("Failed to grab frame")
		break

	# Undistort the frame
	undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

	# Convert to grayscale for detection
	gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

	# Detect AprilTags
	corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)

	if ids is not None:
		for i, corner in enumerate(corners):
			# Solve pose
			success, rvec, tvec = cv2.solvePnP(tag_corners_3d, corner[0], camera_matrix, dist_coeffs)
			if success:
				# Draw markers and axes
				cv2.aruco.drawDetectedMarkers(undistorted, corners, ids)
				cv2.drawFrameAxes(undistorted, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

				# Calculate radial distance
				tag_center_x = np.mean(corner[0][:, 0])
				tag_center_y = np.mean(corner[0][:, 1])
				radial_distance_pixels = np.sqrt((tag_center_x - camera_matrix[0, 2])**2 +
				                                 (tag_center_y - camera_matrix[1, 2])**2)

<<<<<<< HEAD
			print(tvec)

			# Draw axes on the tag to indicate orientation and position
			cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
=======
				# Retrieve raw Z-distance
				raw_distance = tvec[2][0]  # Z-distance in meters
>>>>>>> dfd2eadb5679d3088169354329f397fb858c85e6

				# Calculate dynamic adjustment factor with proximity scaling
				adjustment_factor = calculate_adjustment_factor(
					raw_distance, radial_distance_pixels, max_radius, base_adjustment_factor, reference_distance, k_proximity
				)

				# Apply corrections
				final_corrected_distance = raw_distance * adjustment_factor

				# Debugging output
				print(f"Raw: {raw_distance:.2f}m, Adjustment Factor: {adjustment_factor:.3f}, "
				      f"Corrected: {final_corrected_distance:.2f}m")

				# Display tag information
				text = (f"ID: {ids[i][0]}, Raw Z: {raw_distance:.2f}m, "
				        f"Corrected: {final_corrected_distance:.2f}m")
				cv2.putText(undistorted, text, (10, 50 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

	# Display the frame
	cv2.imshow('AprilTag Detection with Adjustment Factor Scaling', undistorted)

	# Exit on 'q' key
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release resources
cap.release()
cv2.destroyAllWindows()
