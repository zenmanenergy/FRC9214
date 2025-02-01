import cv2
import numpy as np
import os
import csv
from scipy.interpolate import griddata

# ✅ Load camera calibration data
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(os.path.join(current_dir, "calibration", "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "calibration", "dist_coeffs.npy"))

# ✅ Load correction data from CSV
csv_file = os.path.join(current_dir, "distance_data.csv")
correction_data = []

with open(csv_file, "r") as file:
	reader = csv.reader(file)
	next(reader)  # Skip header
	for row in reader:
		yaw_val = float(row[0])
		vertical_val = float(row[1])
		measured_dist = float(row[2])
		actual_dist = float(row[3])
		error = actual_dist - measured_dist  # Calculate distance error
		correction_data.append((yaw_val, vertical_val, error))

correction_data = np.array(correction_data)

# ✅ Define AprilTag detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
parameters = cv2.aruco.DetectorParameters()

# ✅ Open the camera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

print("Starting live preview...")

while True:
	# ✅ Capture frame
	ret, frame = cap.read()
	if not ret:
		print("Failed to grab frame")
		break

	# ✅ Convert to grayscale
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# ✅ Detect AprilTags
	corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

	if ids is not None:
		for i in range(len(ids)):
			# ✅ Get detected corners
			tag_corners = corners[i][0]

			# ✅ SolvePnP for rotation & depth estimation
			ret, rvec, tvec = cv2.solvePnP(
				np.array([
					[-0.082, -0.082, 0], [0.082, -0.082, 0],
					[0.082, 0.082, 0], [-0.082, 0.082, 0]
				], dtype=np.float32), 
				tag_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

			if ret:
				# ✅ Convert rotation vector to rotation matrix
				rmat, _ = cv2.Rodrigues(rvec)

				# ✅ Extract yaw (Rotation around Y-axis)
				yaw = np.degrees(np.arctan2(-rmat[0, 2], rmat[2, 2]))

				# ✅ Compute vertical angle (from screen position)
				image_center_y = camera_matrix[1, 2]  # Cy
				focal_length_y = camera_matrix[1, 1]  # Fy
				tag_center_y = np.mean(tag_corners[:, 1])
				vertical_angle = np.degrees(np.arctan2(tag_center_y - image_center_y, focal_length_y))

				# ✅ Get measured distance from SolvePnP
				measured_distance = float(tvec[2, 0])

				# ✅ Interpolate correction factor from recorded data
				if correction_data.size > 0:
					correction_factor = griddata(
						(correction_data[:, 0], correction_data[:, 1]),
						correction_data[:, 2],
						(yaw, vertical_angle),
						method='linear',
						fill_value=0  # Default to 0 correction if outside range
					)
				else:
					correction_factor = 0

				# ✅ Apply correction to the measured distance
				corrected_distance = measured_distance + correction_factor

				# ✅ Draw bounding box around AprilTag
				cv2.polylines(frame, [np.int32(tag_corners)], isClosed=True, color=(0, 255, 0), thickness=2)

				# ✅ Display yaw, vertical angle, and corrected distance
				center = np.mean(tag_corners, axis=0).astype(int)
				info_text = f"Yaw: {yaw:.1f}° | Vertical: {vertical_angle:.1f}° | Dist: {corrected_distance:.2f}m"
				cv2.putText(frame, info_text, (center[0] - 100, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

	cv2.imshow("AprilTag Detection", frame)

	# ✅ Quit with 'q'
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break

# ✅ Properly close resources
cap.release()
cv2.destroyAllWindows()

print("✅ Live tracking complete.")
