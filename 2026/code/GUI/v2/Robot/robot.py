#!/usr/bin/env python3

import json
import math
import ntcore
import wpilib


class Robot(wpilib.TimedRobot):
	def robotInit(self) -> None:
		self.nt = ntcore.NetworkTableInstance.getDefault()
		self.table = self.nt.getTable("SmartDashboard")
		
		# Robot position and heading
		self.robot_x = 0.0
		self.robot_y = 0.0
		self.robot_heading = 0.0
		
		# Simulation parameters
		self.move_speed = 0.1  # m per cycle
		self.rotate_speed = 2.0  # degrees per cycle
		self.cycle_count = 0

	def _update_and_publish(self) -> None:
		"""Update position and publish to NetworkTables."""
		self.cycle_count += 1
		
		# Move in a circular pattern
		angle_rad = math.radians(self.robot_heading)
		self.robot_x += self.move_speed * math.cos(angle_rad)
		self.robot_y += self.move_speed * math.sin(angle_rad)
		self.robot_heading += self.rotate_speed
		
		# Keep heading in 0-360 range
		self.robot_heading = self.robot_heading % 360
		
		# Send robot state to browser
		robot_state = {
			"x": round(self.robot_x, 2),
			"y": round(self.robot_y, 2),
			"heading": round(self.robot_heading, 1),
			"robot_number": 9214,
			"color": "red",
			"other_robots": []
		}
		self.table.putString("robotData", json.dumps(robot_state))

	
	def teleopPeriodic(self) -> None:
		"""Simulate robot movement during teleop."""
		self._update_and_publish()

	def robotPeriodic(self) -> None:
		"""Always publish, regardless of mode."""
		self._update_and_publish()


if __name__ == "__main__":
	wpilib.run(Robot)
