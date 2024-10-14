import os
import cv2
import numpy as np
import math

# Load calibration data (camera_matrix and dist_coeffs for each camera)
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(os.path.join(current_dir, "calibration", "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "calibration", "dist_coeffs.npy"))

# Define Aruco dictionary and parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

# Camera rig settings
camera_height = 0.755  # Camera height in meters (75.5 cm)
rig_radius = 0.25  # Radius of the 16-sided polygon (25 cm)
camera_offset_angles = [-22.5, 0, 22.5]  # Offset angles for the left, middle, and right cameras

# Define the marker size (in meters)
marker_size = 0.16  # Example: 16 cm markers

# Convert degrees to radians
def deg2rad(deg):
	return deg * (math.pi / 180)

# Calculate the transformation matrix for the offset
def calculate_transformation_matrix(angle_deg):
	angle_rad = deg2rad(angle_deg)
	cos_angle = np.cos(angle_rad)
	sin_angle = np.sin(angle_rad)
	
	# Rotation matrix (rotation around Z-axis)
	return np.array([
		[cos_angle, -sin_angle, 0],
		[sin_angle, cos_angle, 0],
		[0, 0, 1]
	])

# Detect and estimate positions of markers in world coordinates
def detect_and_estimate_position(frame, camera_matrix, dist_coeffs, rotation_matrix):
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	# Detect markers
	corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
	
	if ids is not None:
		# Estimate the pose of each marker
		rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
		
		# Loop through all detected markers
		for i, id in enumerate(ids):
			rvec = rvecs[i]
			tvec = tvecs[i]
			
			# Draw marker and axis on the frame for visualization
			cv2.aruco.drawDetectedMarkers(frame, corners)
			cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
			
			# Transform tvec to world coordinates using the camera's rotation matrix
			tvec_world = rotation_matrix @ tvec.T
			x, y, z = tvec_world.flatten()
			
			# Adjust the z coordinate to account for the camera's height above the ground
			z += camera_height
			
			print(f"Marker ID {id[0]}: x={x:.2f}, y={y:.2f}, z={z:.2f}")
			
			# If ID=0, set it as the origin (0, 0, z)
			if id[0] == 0:
				origin_z = z
				print(f"ID=0 origin set at (0, 0, {origin_z:.2f})")
				x, y, z = 0, 0, origin_z

	return frame

def main():
	# Initialize video capture for three cameras
	caps = [cv2.VideoCapture(i) for i in range(3)]  # Cameras 0, 1, and 2

	# Calculate the transformation matrices for each camera's rotation
	rotation_matrices = [calculate_transformation_matrix(angle) for angle in camera_offset_angles]

	while True:
		for idx, cap in enumerate(caps):
			ret, frame = cap.read()
			if not ret:
				continue

			# Detect and estimate marker positions for each camera
			frame = detect_and_estimate_position(frame, camera_matrix, dist_coeffs, rotation_matrices[idx])

			# Display the frame with detected markers
			cv2.imshow(f'Aruco Detection - Camera {idx}', frame)

		# Exit if 'q' is pressed
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	# Release resources
	for cap in caps:
		cap.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()
