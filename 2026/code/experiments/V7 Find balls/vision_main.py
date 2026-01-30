import cv2
import time
from ball_cluster_vision import BallClusterVision


def main():
	# ---------------- Cameras ----------------
	# Adjust IDs if needed (USB ordering matters)
	left_cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
	right_cam = cv2.VideoCapture(1, cv2.CAP_DSHOW)

	left_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
	left_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
	right_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
	right_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

	# ---------------- Vision System ----------------
	vision = BallClusterVision(server_ip="10.XX.YY.2")

	# ---------------- Main Loop ----------------
	while True:
		ret_l, left_frame = left_cam.read()
		ret_r, right_frame = right_cam.read()

		if not ret_l or not ret_r:
			time.sleep(0.02)
			continue

		vision.update(left_frame, right_frame)

		# Run ~20 Hz
		time.sleep(0.05)


if __name__ == "__main__":
	main()
