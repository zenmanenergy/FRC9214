import cv2
import numpy as np
import os
import csv


# ✅ Load camera calibration data
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(os.path.join(current_dir, "calibration", "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "calibration", "dist_coeffs.npy"))

# ✅ Define AprilTag properties
APRILTAG_SIZE = 0.164  # 164mm (real-world size in meters)

# ✅ Define object points for the AprilTag (3D real-world coordinates)
obj_points = np.array([
	[-APRILTAG_SIZE / 2, -APRILTAG_SIZE / 2, 0],
	[ APRILTAG_SIZE / 2, -APRILTAG_SIZE / 2, 0],
	[ APRILTAG_SIZE / 2,  APRILTAG_SIZE / 2, 0],
	[-APRILTAG_SIZE / 2,  APRILTAG_SIZE / 2, 0]
], dtype=np.float32)

# ✅ Define AprilTag detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
parameters = cv2.aruco.DetectorParameters()

# ✅ Open the camera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
image_size = (1920, 1080)

# ✅ Open CSV file for writing (Keep it open)
csv_file = os.path.join(current_dir, "distance_data.csv")
file_exists = os.path.exists(csv_file)
csv_f = open(csv_file, "a", newline="")  # Keep file open
writer = csv.writer(csv_f)

if not file_exists:
	writer.writerow(["Yaw (°)", "vertical_angle (°)", "Measured Distance (m)", "Actual Distance (m)"])

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

			# ✅ Use `solvePnP()` for rotation & depth estimation
			ret, rvec, tvec = cv2.solvePnP(obj_points, tag_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

			if ret:
				# Convert rotation vector to rotation matrix
				# Convert rotation vector to rotation matrix
				rmat, _ = cv2.Rodrigues(rvec)

				yaw = np.degrees(np.arctan2(-rmat[0, 2], rmat[2, 2]))   # Rotation around Y-axis
				# Get image center and focal length from camera calibration
				image_center_y = camera_matrix[1, 2]  # Cy (camera principal point y)
				focal_length_y = camera_matrix[1, 1]  # Fy (focal length in y direction)

				# Compute the center of the detected AprilTag
				tag_center_y = np.mean(tag_corners[:, 1])  # Average Y-coordinate of detected corners

				# Compute vertical angle from image center
				vertical_angle = np.degrees(np.arctan2(tag_center_y - image_center_y, focal_length_y))



				# yaw=np.degrees(np.arcsin(-rmat[2, 0]))
				measured_distance = float(tvec[2, 0])

				# ✅ Draw bounding box around AprilTag
				cv2.polylines(frame, [np.int32(tag_corners)], isClosed=True, color=(0, 255, 0), thickness=2)

				# ✅ Display yaw, vertical_angle, and distance on the frame
				center = np.mean(tag_corners, axis=0).astype(int)
				info_text = f"Yaw: {yaw:.1f}° | vertical_angle: {vertical_angle:.1f}° | Dist: {measured_distance:.2f}m"
				cv2.putText(frame, info_text, (center[0] - 100, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

	cv2.imshow("AprilTag Detection", frame)

	# ✅ Wait for user input to record a measurement
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):  # Quit
		break
	elif key == ord("s") and ids is not None:  # Save data
		# ✅ Ask for the actual distance
		while True:
			actual_distance = input("Enter actual distance in cm (or type 'q' to quit): ").strip()
			if actual_distance.lower() == "q":
				print("Exiting data collection.")
				break

			try:
				actual_distance = float(actual_distance)/100
				break
			except ValueError:
				print("Invalid input. Enter a valid distance in cm.")

		# ✅ Save data to CSV
		writer.writerow([yaw, vertical_angle, measured_distance, actual_distance])
		csv_f.flush()  # ✅ Ensure data is written immediately
		print(f"✅ Saved: Yaw={yaw:.2f}°, vertical_angle={vertical_angle:.2f}°, Measured={measured_distance:.3f}m, Actual={actual_distance:.3f}cm")

# ✅ Properly close resources
cap.release()
cv2.destroyAllWindows()
csv_f.close()

print("✅ Data collection complete.")
