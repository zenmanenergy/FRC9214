import cv2
import numpy as np
import os
import csv
import math

# ✅ AprilTag field coordinates (converted to cm)
APRILTAG_MAP = {
	1: {"x": 657.37 * 2.54, "y": 25.80 * 2.54, "z_rotation": 126},
	2: {"x": 657.37 * 2.54, "y": 291.20 * 2.54, "z_rotation": 234},
	3: {"x": 455.15 * 2.54, "y": 317.15 * 2.54, "z_rotation": 270},
	4: {"x": 365.20 * 2.54, "y": 241.64 * 2.54, "z_rotation": 0},
	5: {"x": 365.20 * 2.54, "y": 75.39 * 2.54, "z_rotation": 0},
	6: {"x": 530.49 * 2.54, "y": 130.17 * 2.54, "z_rotation": 300},
	7: {"x": 546.87 * 2.54, "y": 158.50 * 2.54, "z_rotation": 0},
	8: {"x": 530.49 * 2.54, "y": 186.83 * 2.54, "z_rotation": 60},
	9: {"x": 497.77 * 2.54, "y": 186.83 * 2.54, "z_rotation": 120},
	10: {"x": 481.39 * 2.54, "y": 158.50 * 2.54, "z_rotation": 180},
	11: {"x": 497.77 * 2.54, "y": 130.17 * 2.54, "z_rotation": 240},
	12: {"x": 33.51 * 2.54, "y": 25.80 * 2.54, "z_rotation": 54},
	13: {"x": 33.51 * 2.54, "y": 291.20 * 2.54, "z_rotation": 306},
	14: {"x": 325.68 * 2.54, "y": 241.64 * 2.54, "z_rotation": 180},
	15: {"x": 325.68 * 2.54, "y": 75.39 * 2.54, "z_rotation": 180},
	16: {"x": 235.73 * 2.54, "y": -0.15 * 2.54, "z_rotation": 90},
	17: {"x": 160.39 * 2.54, "y": 130.17 * 2.54, "z_rotation": 240},
	18: {"x": 144.00 * 2.54, "y": 158.50 * 2.54, "z_rotation": 180},
	19: {"x": 160.39 * 2.54, "y": 186.83 * 2.54, "z_rotation": 120},
	20: {"x": 193.10 * 2.54, "y": 186.83 * 2.54, "z_rotation": 60},
	21: {"x": 209.49 * 2.54, "y": 158.50 * 2.54, "z_rotation": 0},
	22: {"x": 193.10 * 2.54, "y": 130.17 * 2.54, "z_rotation": 300}
}

# ✅ Function to compute robot position from tag data
def compute_robot_position(tag_id, measured_distance_cm, measured_yaw):
	if tag_id not in APRILTAG_MAP:
		return None  # Ignore unknown tags
	
	tag = APRILTAG_MAP[tag_id]
	tag_x, tag_y = tag["x"], tag["y"]
	tag_z_rotation = tag["z_rotation"]

	# ✅ Compute the global yaw of the robot
	global_yaw = (tag_z_rotation + measured_yaw) % 360  # Normalize to [0, 360]
	global_yaw_rad = math.radians(global_yaw)  # Convert to radians for calculations

	# ✅ Compute robot's position using trigonometry
	robot_x = tag_x - measured_distance_cm * math.cos(global_yaw_rad)
	robot_y = tag_y - measured_distance_cm * math.sin(global_yaw_rad)

	return robot_x, robot_y, global_yaw

# ✅ Load camera calibration data
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(os.path.join(current_dir, "calibration", "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "calibration", "dist_coeffs.npy"))

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
			tag_id = int(ids[i][0])
			tag_corners = corners[i][0]

			# ✅ SolvePnP for rotation & depth estimation
			ret, rvec, tvec = cv2.solvePnP(
				np.array([[-0.082, -0.082, 0], [0.082, -0.082, 0],
						  [0.082, 0.082, 0], [-0.082, 0.082, 0]], dtype=np.float32),
				tag_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

			if ret:
				# ✅ Convert rotation vector to rotation matrix
				rmat, _ = cv2.Rodrigues(rvec)

				# ✅ Extract yaw (Rotation around Y-axis)
				measured_yaw = np.degrees(np.arctan2(-rmat[0, 2], rmat[2, 2]))

				# ✅ Convert measured distance from meters to cm
				measured_distance_cm = float(tvec[2, 0]) * 100

				# ✅ Compute robot position
				robot_pos = compute_robot_position(tag_id, measured_distance_cm, measured_yaw)
				if robot_pos:
					robot_x, robot_y, robot_heading = robot_pos

					# ✅ Draw bounding box around AprilTag
					cv2.polylines(frame, [np.int32(tag_corners)], isClosed=True, color=(0, 255, 0), thickness=2)

					# ✅ Display info on the image
					center = np.mean(tag_corners, axis=0).astype(int)
					info_text = f"ID: {tag_id} | X: {robot_x:.2f} cm | Y: {robot_y:.2f} cm | Heading: {robot_heading:.1f}°"
					cv2.putText(frame, info_text, (center[0] - 100, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

					# ✅ Show distance to tag
					distance_text = f"Dist: {measured_distance_cm:.1f} cm"
					cv2.putText(frame, distance_text, (center[0] - 50, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

					# ✅ Print for debugging
					print(f"Robot Position: X={robot_x:.2f} cm, Y={robot_y:.2f} cm, Heading={robot_heading:.1f}°")

	cv2.imshow("AprilTag Detection", frame)

	# ✅ Quit with 'q'
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break

# ✅ Properly close resources
cap.release()
cv2.destroyAllWindows()

print("✅ Live tracking complete.")
