"""
NavX Rotate to Angle Example

This example demonstrates using the navX MXP to implement a 
"rotate to angle" feature.

The robot will automatically rotate to one of four angles 
(0, 90, 180, and 270 degrees) when corresponding buttons are pressed.

Rotation can occur while the robot is still or while driving. 
When using field-oriented control, this causes the robot to drive 
in a straight line in the selected direction.

The example also includes a feature to reset the gyro yaw angle to 0 degrees,
useful when gyro drift occurs during long practice sessions.

Note: PID Controller coefficients will need tuning for your drive system.
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

	# PID Controller coefficients (tune for your drive system)
	kP = 0.03
	kI = 0.00
	kD = 0.00
	kF = 0.00
	
	# Tolerance for rotation
	TOLERANCE_DEGREES = 2.0

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

		# Initialize PID controller for rotation
		self.turn_controller = wpilib.PIDController(
			self.kP, self.kI, self.kD,
			self._get_yaw,
			self._set_rotation_rate
		)
		self.turn_controller.setInputRange(-180.0, 180.0)
		self.turn_controller.setOutputRange(-1.0, 1.0)
		self.turn_controller.setAbsoluteTolerance(self.TOLERANCE_DEGREES)
		self.turn_controller.setContinuous(True)

		# Rotation rate from PID controller
		self.rotate_to_angle_rate = 0.0

	def _get_yaw(self) -> float:
		"""Get current yaw angle from navX."""
		if self.ahrs:
			return self.ahrs.getAngle()
		return 0.0

	def _set_rotation_rate(self, output: float) -> None:
		"""Callback for PID controller output."""
		self.rotate_to_angle_rate = output

	def autonomousInit(self):
		"""Initialize autonomous mode."""
		self.my_robot.setSafetyEnabled(False)

	def autonomousPeriodic(self):
		"""Autonomous does nothing."""
		self.my_robot.driveCartesian(0.0, 0.0, 0.0)

	def teleopInit(self):
		"""Initialize teleop mode."""
		self.my_robot.setSafetyEnabled(True)

	def teleopPeriodic(self):
		"""Handle joystick input and field-centric drive with rotate to angle."""
		if not self.ahrs:
			return

		rotate_to_angle = False

		# Button 1: Reset gyro
		if self.stick.getRawButton(1):
			self.ahrs.reset()

		# Buttons 2-5: Rotate to angle
		if self.stick.getRawButton(2):
			self.turn_controller.setSetpoint(0.0)
			rotate_to_angle = True
		elif self.stick.getRawButton(3):
			self.turn_controller.setSetpoint(90.0)
			rotate_to_angle = True
		elif self.stick.getRawButton(4):
			self.turn_controller.setSetpoint(179.9)
			rotate_to_angle = True
		elif self.stick.getRawButton(5):
			self.turn_controller.setSetpoint(-90.0)
			rotate_to_angle = True

		# Set rotation rate based on rotate-to-angle mode
		if rotate_to_angle:
			self.turn_controller.enable()
			current_rotation_rate = self.rotate_to_angle_rate
		else:
			self.turn_controller.disable()
			current_rotation_rate = self.stick.getTwist()

		# Drive with field-centric control
		try:
			self.my_robot.driveCartesian(
				self.stick.getX(),
				self.stick.getY(),
				current_rotation_rate,
				self.ahrs.getAngle()
			)
		except RuntimeError as ex:
			wpilib.reportError(f"Error communicating with drive system: {ex}", True)


if __name__ == "__main__":
	wpilib.run(Robot)
