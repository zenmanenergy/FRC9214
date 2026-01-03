#!/usr/bin/env python3

import time
import json
from ntcore import NetworkTableInstance

def main():
	ntinst = NetworkTableInstance.getDefault()
	ntinst.startClient4("coprocessor")
	ntinst.setServer("10.92.14.2")  # Change to your RoboRIO IP or MDNS name

	table = ntinst.getTable("SmartDashboard")

	def on_target_click(topic):
		"""Callback for receiving target coordinate clicks from GUI."""
		target_data = json.loads(topic.value)
		print(f"Received target: x={target_data['x']}, y={target_data['y']}")

	table.getEntry("targetCoordinate").addListener(on_target_click)

	while True:
		# Replace with actual sensor/vision data
		robot_state = {
			"timestamp": time.time(),
			"x": 3.5,
			"y": 2.1,
			"heading": 45.0,
			"robot_number": 9214,
			"color": "red",
			"other_robots": [
				{"x": 6.2, "y": 1.8, "heading": 90.0, "robot_number": 5678, "color": "blue"},
				{"x": 2.3, "y": 5.0, "heading": 270.0, "robot_number": 9012, "color": "red"}
			]
		}

		# Publish as JSON string
		table.putString("robotData", json.dumps(robot_state))

		time.sleep(0.05)  # 20Hz update rate

if __name__ == "__main__":
	main()
