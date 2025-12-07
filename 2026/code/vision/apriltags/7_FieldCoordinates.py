import cv2
import numpy as np
import os
import math
import json
from ntcore import NetworkTableInstance

# ✅ Initialize NetworkTables
nt_inst = NetworkTableInstance.getDefault()
nt_inst.setServer("10.92.14.2")  # Replace with your RoboRIO's IP
nt_inst.startClient4("VisionSystem")
nt_inst.startDSClient()
vision_table = nt_inst.getTable("Vision")

# ✅ AprilTag field coordinates (in cm)
APRILTAG_MAP = {
	1: {"x": 657.37 * 2.54, "y": 25.80 * 2.54, "z_rotation": 126},
	2: {"x": 657.37 * 2.54, "y": 291.20 * 2.54, "z_rotation": 234},
	3: {"x": 455.15 * 2.54, "y": 317.15 * 2.54, "z_rotation": 270},
	4: {"x": 365.20 * 2.54, "y": 241.64 * 2.54, "z_rotation": 0},
	5: {"x": 365.20 * 2.54, "y": 75.39 * 2.54, "z_rotation": 0}
}

# ✅ Function to compute robot position based on detected tag
def compute_robot_position(tag_id, measured_distance_cm, measured_yaw):
	if tag_id not in APRILTAG_MAP:
		return None  
	
	tag = APRILTAG_MAP[tag_id]
	tag_x, tag_y = tag["x"], tag["y"]
	tag_z_rotation = tag["z_rotation"]

	# ✅ Compute global yaw
	global_yaw = (tag_z_rotation + measured_yaw) % 360  
	global_yaw_rad = math.radians(global_yaw)  

	# ✅ Compute robot position in cm
	robot_x = tag_x - measured_distance_cm * math.cos(global_yaw_rad)
	robot_y = tag_y - measured_distance_cm * math.sin(global_yaw_rad)

	return robot_x, robot_y, global_yaw

# ✅ Load camera calibration data
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(os.path.join(current_dir, "calibration", "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "calibration", "dist_coeffs.npy"))

# ✅ Open the camera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

print("Starting live preview...")

while True:
	ret, frame = cap.read()
	if not ret:
		print("Failed to grab frame")
		break

	# ✅ Convert to grayscale
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	corners, ids, _ = cv2.aruco.detectMarkers(gray, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))

	if ids is not None:
		for i in range(len(ids)):
			tag_id = int(ids[i][0])
			tag_corners = corners[i][0]

			# ✅ SolvePnP for position estimation
			ret, rvec, tvec = cv2.solvePnP(
				np.array([[-0.082, -0.082, 0], [0.082, -0.082, 0],
						  [0.082, 0.082, 0], [-0.082, 0.082, 0]], dtype=np.float32),
				tag_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

			if ret:
				# ✅ Convert rotation vector to rotation matrix
				rmat, _ = cv2.Rodrigues(rvec)
				measured_yaw = np.degrees(np.arctan2(-rmat[0, 2], rmat[2, 2]))

				# ✅ Convert measured distance from meters to cm (No Changes)
				measured_distance_cm = float(tvec[2, 0]) * 100  

				# ✅ Compute robot position
				robot_pos = compute_robot_position(tag_id, measured_distance_cm, measured_yaw)
				if robot_pos:
					robot_x, robot_y, robot_heading = robot_pos

					# ✅ Send data to NetworkTables
					data = {
						"x": round(robot_x, 2),
						"y": round(robot_y, 2),
						"heading": round(robot_heading, 1),
						"distance_to_tag": round(measured_distance_cm, 1)
					}
					vision_table.putString("robot_position", json.dumps(data))

					# ✅ Draw bounding box
					cv2.polylines(frame, [np.int32(tag_corners)], isClosed=True, color=(0, 255, 0), thickness=2)

					# ✅ Display information on image
					center = np.mean(tag_corners, axis=0).astype(int)
					info_text = f"X: {robot_x:.2f} cm | Y: {robot_y:.2f} cm | Heading: {robot_heading:.1f}°"
					cv2.putText(frame, info_text, (center[0] - 100, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
					cv2.putText(frame, f"Dist: {measured_distance_cm:.1f} cm", (center[0] - 50, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

					# ✅ Debugging Output
					print(f"Robot Position: X={robot_x:.2f} cm, Y={robot_y:.2f} cm, Heading={robot_heading:.1f}°, Distance={measured_distance_cm:.1f} cm")

	cv2.imshow("AprilTag Detection", frame)
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break

cap.release()
cv2.destroyAllWindows()
nt_inst.stopClient()
