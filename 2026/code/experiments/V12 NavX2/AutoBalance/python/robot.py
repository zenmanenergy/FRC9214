"""
NavX Auto Balance Example

This example demonstrates using the navX MXP to implement an automatic 
balance feature to help avoid a robot tipping over when driving.

The basic principle is measurement of the Pitch (rotation about X axis) 
and Roll (rotation about Y axis) angles. When these angles exceed the 
"off balance" threshold and until they fall below the "on balance" 
threshold, the drive system is automatically driven in the opposite 
direction at a magnitude proportional to the Pitch or Roll angle.

Note: This is a starting point for automatic balancing and will likely 
require tuning for your specific robot.
"""

import math
import wpilib
from navx import AHRS


class Robot(wpilib.TimedRobot):
	# Drive channel assignments
	FRONT_LEFT_CHANNEL = 2
	REAR_LEFT_CHANNEL = 3
	FRONT_RIGHT_CHANNEL = 1
	REAR_RIGHT_CHANNEL = 0

	# Auto-balance thresholds
	OFF_BALANCE_ANGLE_THRESHOLD = 10.0  # degrees
	ON_BALANCE_ANGLE_THRESHOLD = 5.0    # degrees

	def robotInit(self):
		"""Initialize robot components."""
		# Initialize motor controllers
		self.front_left = wpilib.Spark(self.FRONT_LEFT_CHANNEL)
		self.rear_left = wpilib.Spark(self.REAR_LEFT_CHANNEL)
		self.front_right = wpilib.Spark(self.FRONT_RIGHT_CHANNEL)
		self.rear_right = wpilib.Spark(self.REAR_RIGHT_CHANNEL)

		# Initialize drive system
		self.my_robot = wpilib.drive.MecanumDrive(
			self.front_left, self.rear_left,
			self.front_right, self.rear_right
		)
		self.my_robot.setExpiration(0.1)

		# Initialize joystick
		self.stick = wpilib.Joystick(0)

		# Initialize navX
		try:
			self.ahrs = AHRS.create_spi()
		except RuntimeError as ex:
			wpilib.reportError(f"Error instantiating navX MXP: {ex}", True)
			self.ahrs = None

		# Auto-balance state
		self.auto_balance_x_mode = False
		self.auto_balance_y_mode = False

	def autonomousInit(self):
		"""Initialize autonomous mode."""
		self.my_robot.setSafetyEnabled(False)

	def autonomousPeriodic(self):
		"""Drive forward for 2 seconds then stop."""
		self.my_robot.driveCartesian(0.0, -0.5, 0.0)
		if self.getAutonomousTime() >= 2.0:
			self.my_robot.driveCartesian(0.0, 0.0, 0.0)

	def teleopInit(self):
		"""Initialize teleop mode."""
		self.my_robot.setSafetyEnabled(True)

	def teleopPeriodic(self):
		"""Process joystick input and auto-balance."""
		if not self.ahrs:
			return

		x_axis_rate = self.stick.getX()
		y_axis_rate = self.stick.getY()
		pitch_angle = self.ahrs.getPitch()
		roll_angle = self.ahrs.getRoll()

		# Update auto-balance X mode based on pitch angle
		if not self.auto_balance_x_mode:
			if abs(pitch_angle) >= self.OFF_BALANCE_ANGLE_THRESHOLD:
				self.auto_balance_x_mode = True
		else:
			if abs(pitch_angle) <= self.ON_BALANCE_ANGLE_THRESHOLD:
				self.auto_balance_x_mode = False

		# Update auto-balance Y mode based on roll angle
		if not self.auto_balance_y_mode:
			if abs(roll_angle) >= self.OFF_BALANCE_ANGLE_THRESHOLD:
				self.auto_balance_y_mode = True
		else:
			if abs(roll_angle) <= self.ON_BALANCE_ANGLE_THRESHOLD:
				self.auto_balance_y_mode = False

		# Apply auto-balance corrections
		if self.auto_balance_x_mode:
			pitch_radians = math.radians(pitch_angle)
			x_axis_rate = math.sin(pitch_radians) * -1.0

		if self.auto_balance_y_mode:
			roll_radians = math.radians(roll_angle)
			y_axis_rate = math.sin(roll_radians) * -1.0

		# Drive the robot
		try:
			self.my_robot.driveCartesian(x_axis_rate, y_axis_rate, self.stick.getTwist(), 0)
		except RuntimeError as ex:
			wpilib.reportError(f"Drive system error: {ex}", True)


if __name__ == "__main__":
	wpilib.run(Robot)
