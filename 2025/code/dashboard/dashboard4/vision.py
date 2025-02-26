import cv2
import numpy as np
import os
import math
import time
from networktables import NetworkTables

# ✅ AprilTag Map (Field Positions in cm)
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

# ✅ Camera Offsets (x, y, height)
CAMERA_OFFSETS = {
	"FL": (0, 0, 47),     
	"FR": (51, 0, 47),    
	"BL": (0, -67, 47),   
	"BR": (51, -67, 47),  
}

# ✅ Load Camera Calibration Data
current_dir = os.path.dirname(os.path.abspath(__file__))
camera_matrix = np.load(os.path.join(current_dir, "calibration", "camera_matrix.npy"))
dist_coeffs = np.load(os.path.join(current_dir, "calibration", "dist_coeffs.npy"))

# ✅ Setup AprilTag Detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
parameters = cv2.aruco.DetectorParameters()

# ✅ Initialize NetworkTables
table = NetworkTables.getTable("robot_data")

# ✅ Function to Compute Robot Position
def compute_robot_position(tag_id, measured_distance_cm, measured_yaw):
	if tag_id not in APRILTAG_MAP:
		return None  

	tag = APRILTAG_MAP[tag_id]
	tag_x, tag_y = tag["x"], tag["y"]
	tag_z_rotation = tag["z_rotation"]

	# ✅ Compute Global Yaw
	global_yaw = (tag_z_rotation + measured_yaw) % 360
	global_yaw_rad = math.radians(global_yaw)

	# ✅ Compute Robot Position
	robot_x = tag_x - measured_distance_cm * math.cos(global_yaw_rad)
	robot_y = tag_y - measured_distance_cm * math.sin(global_yaw_rad)

	return robot_x, robot_y, global_yaw

# ✅ Function to Run Vision Processing
def vision_loop():
	# ✅ Initialize Cameras
	cameras = {
		"FL": cv2.VideoCapture(0, cv2.CAP_DSHOW),
		"FR": cv2.VideoCapture(1, cv2.CAP_DSHOW),
		"BL": cv2.VideoCapture(2, cv2.CAP_DSHOW),
		"BR": cv2.VideoCapture(3, cv2.CAP_DSHOW),
	}

	for cap in cameras.values():
		cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

	print("Vision processing started...")

	while True:
		camera_readings = {}

		for cam_id, cap in cameras.items():
			ret, frame = cap.read()
			if not ret:
				continue

			# ✅ Convert to Grayscale
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			# ✅ Detect AprilTags
			corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

			if ids is not None:
				for i in range(len(ids)):
					tag_id = int(ids[i][0])
					tag_corners = corners[i][0]

					# ✅ SolvePnP for Pose Estimation
					ret, rvec, tvec = cv2.solvePnP(
						np.array([[-0.082, -0.082, 0], [0.082, -0.082, 0],
								[0.082, 0.082, 0], [-0.082, 0.082, 0]], dtype=np.float32),
						tag_corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

					if ret:
						# ✅ Extract Yaw
						rmat, _ = cv2.Rodrigues(rvec)
						measured_yaw = np.degrees(np.arctan2(-rmat[0, 2], rmat[2, 2]))

						# ✅ Correct Distance for Camera Height
						cam_x_offset, cam_y_offset, cam_height = CAMERA_OFFSETS[cam_id]
						raw_distance_cm = float(tvec[2, 0]) * 100  
						measured_distance_cm = math.sqrt(raw_distance_cm**2 - cam_height**2) if raw_distance_cm > cam_height else raw_distance_cm

						# ✅ Compute Robot Position
						robot_pos = compute_robot_position(tag_id, measured_distance_cm, measured_yaw)
						if robot_pos:
							camera_readings[cam_id] = robot_pos

		# ✅ Every 5 seconds, update NetworkTables
		time.sleep(5)
		if camera_readings:
			final_x = sum([p[0] for p in camera_readings.values()]) / len(camera_readings)
			final_y = sum([p[1] for p in camera_readings.values()]) / len(camera_readings)
			final_heading = sum([p[2] for p in camera_readings.values()]) / len(camera_readings)

			# ✅ Send the computed position to NetworkTables (RoboRIO will handle broadcasting)
			table.putNumber("vision_x_position", final_x)
			table.putNumber("vision_y_position", final_y)
			table.putNumber("vision_heading", final_heading)

			print(f"Updated Vision Position: X={final_x:.2f} cm, Y={final_y:.2f} cm, Heading={final_heading:.1f}°")

	for cap in cameras.values():
		cap.release()