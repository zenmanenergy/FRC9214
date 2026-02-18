"""
NavX Collision Detection Example

This example demonstrates using the navX MXP to implement a collision 
detection feature, which can detect events while driving such as bumping 
into a wall or being hit by another robot.

The basic principle is calculation of Jerk (change in acceleration). 
Both X and Y axis jerk are calculated, and if either exceeds a threshold, 
a collision has occurred.

Note: The collision threshold will likely need tuning for your robot, 
as the jerk that constitutes a collision depends on robot mass and 
expected maximum velocity.
"""

import wpilib
from navx import AHRS


class Robot(wpilib.TimedRobot):
	# Drive channel assignments
	FRONT_LEFT_CHANNEL = 2
	REAR_LEFT_CHANNEL = 3
	FRONT_RIGHT_CHANNEL = 1
	REAR_RIGHT_CHANNEL = 0

	# Collision detection threshold (change in acceleration in Gs)
	COLLISION_THRESHOLD = 0.5

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

		# Collision detection state
		self.last_accel_x = 0.0
		self.last_accel_y = 0.0

	def autonomousInit(self):
		"""Initialize autonomous mode."""
		self.my_robot.setSafetyEnabled(False)

	def autonomousPeriodic(self):
		"""Drive forward for 2 seconds then stop."""
		self.my_robot.driveCartesian(0.0, 0.5, 0.0)
		if self.getAutonomousTime() >= 2.0:
			self.my_robot.driveCartesian(0.0, 0.0, 0.0)

	def teleopInit(self):
		"""Initialize teleop mode."""
		self.my_robot.setSafetyEnabled(True)

	def teleopPeriodic(self):
		"""Detect collisions and drive the robot."""
		if not self.ahrs:
			return

		collision_detected = False

		# Calculate jerk (change in acceleration)
		curr_accel_x = self.ahrs.getWorldLinearAccelX()
		jerk_x = curr_accel_x - self.last_accel_x
		self.last_accel_x = curr_accel_x

		curr_accel_y = self.ahrs.getWorldLinearAccelY()
		jerk_y = curr_accel_y - self.last_accel_y
		self.last_accel_y = curr_accel_y

		# Check if collision threshold exceeded
		if abs(jerk_x) > self.COLLISION_THRESHOLD or abs(jerk_y) > self.COLLISION_THRESHOLD:
			collision_detected = True

		wpilib.SmartDashboard.putBoolean("CollisionDetected", collision_detected)

		# Drive the robot
		try:
			self.my_robot.driveCartesian(self.stick.getX(), self.stick.getY(), self.stick.getTwist(), 0)
		except RuntimeError as ex:
			wpilib.reportError(f"Drive system error: {ex}", True)


if __name__ == "__main__":
	wpilib.run(Robot)
