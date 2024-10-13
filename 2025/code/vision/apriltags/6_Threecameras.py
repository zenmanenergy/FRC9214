import os
import cv2
import apriltag
import numpy as np

# Load calibration data
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(os.path.join(current_dir, "calibration", "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "calibration", "dist_coeffs.npy"))

# Define camera positions and angles in the semi-circle (use polar coordinates)
camera_positions = [
	(14.4, 0),  # Camera 1 at 0 degrees
	(14.4, np.pi / 4),  # Camera 2 at 45 degrees
	(14.4, np.pi / 2)  # Camera 3 at 90 degrees
]

def calculate_tag_pose(corner_points, camera_matrix, dist_coeffs):
	# Prepare object points of the tag (assuming a square tag of known size)
	tag_size = 0.05  # Size of the tag in meters, replace with your tag size
	obj_points = np.array([
		[-tag_size / 2, -tag_size / 2, 0],
		[tag_size / 2, -tag_size / 2, 0],
		[tag_size / 2, tag_size / 2, 0],
		[-tag_size / 2, tag_size / 2, 0]
	], dtype=np.float32)

	# Use solvePnP to calculate the pose of the tag relative to the camera
	ret, rvec, tvec = cv2.solvePnP(obj_points, corner_points, camera_matrix, dist_coeffs)
	return rvec, tvec

def calculate_angles(tvec):
	# Calculate horizontal and vertical angles using the translation vector
	distance = np.linalg.norm(tvec)
	horizontal_angle = np.arctan2(tvec[0], tvec[2])  # Angle in XZ plane
	vertical_angle = np.arctan2(tvec[1], tvec[2])  # Angle in YZ plane
	return distance, horizontal_angle, vertical_angle

def map_tags_relative_to_id0(tag_positions, id0_position):
	# Shift all tags' positions relative to ID=0 (the origin)
	tag_map = {}
	for tag_id, position in tag_positions.items():
		if tag_id == 0:
			continue
		# Translate all tags relative to ID 0's position
		relative_position = position - id0_position
		tag_map[tag_id] = relative_position
	return tag_map

def main():
	# Initialize video capture for each camera
	cap1 = cv2.VideoCapture(0)
	cap2 = cv2.VideoCapture(1)
	cap3 = cv2.VideoCapture(2)

	# Initialize AprilTag detector
	detector = apriltag.Detector()

	# To store tag positions relative to each camera
	tag_positions = {}

	while True:
		for cap, camera_position in zip([cap1, cap2, cap3], camera_positions):
			ret, frame = cap.read()
			if not ret:
				continue

			# Convert frame to grayscale for AprilTag detection
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			# Detect AprilTags
			tags = detector.detect(gray)

			for tag in tags:
				# Get corner points and calculate pose
				corner_points = tag.corners.astype(np.float32)
				rvec, tvec = calculate_tag_pose(corner_points, camera_matrix, dist_coeffs)

				# Calculate distance and angles
				distance, h_angle, v_angle = calculate_angles(tvec)

				# Store the tag position in polar coordinates relative to the camera
				tag_positions[tag.tag_id] = np.array([distance, h_angle, v_angle])

				# Draw the detection on the frame
				cv2.polylines(frame, [corner_points.astype(np.int32)], True, (0, 255, 0), 2)
				cv2.putText(frame, f"ID: {tag.tag_id}", (int(tag.corners[0][0]), int(tag.corners[0][1])),
							cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

			# Display the camera feed
			cv2.imshow(f'Camera {cap}', frame)

		# Check for a specific tag (ID=0) and map other tags relative to it
		if 0 in tag_positions:
			id0_position = tag_positions[0]
			relative_positions = map_tags_relative_to_id0(tag_positions, id0_position)
			print(f"Relative positions of tags: {relative_positions}")

		# Break the loop if 'q' is pressed
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	# Release camera resources
	cap1.release()
	cap2.release()
	cap3.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()
