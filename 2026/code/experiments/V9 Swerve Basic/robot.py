import wpilib

from drive import SwerveDrive, SwerveModule
from joystick_drive import DriverJoystick


class Robot(wpilib.TimedRobot):
	"""Main robot container and teleop code."""

	def robotInit(self):
		"""Initialize robot and subsystems."""
		print("\n===== ROBOT INIT STARTING =====")

		# Initialize joystick
		self.driver = DriverJoystick()

		# Create swerve modules
		print("[INIT] Creating swerve modules...")
		self.front_left = SwerveModule(drive_motor_id=2, turn_motor_id=3)
		self.front_right = SwerveModule(drive_motor_id=8, turn_motor_id=9)
		self.rear_left = SwerveModule(drive_motor_id=4, turn_motor_id=5)
		self.rear_right = SwerveModule(drive_motor_id=6, turn_motor_id=7)
		print("[INIT] Modules created")

		# Create swerve drive
		self.swerve = SwerveDrive(
			self.front_left,
			self.front_right,
			self.rear_left,
			self.rear_right,
		)
		print("===== ROBOT INIT COMPLETE =====\n")

	def teleopInit(self):
		"""Called when entering teleop mode - stop everything."""
		print("[ROBOT] Teleop started - stopping all motors")
		if self.swerve is not None:
			self.swerve.stop()

	def teleopPeriodic(self):
		"""Handle teleop control inputs."""
		if self.swerve is None:
			return

		# Get joystick inputs
		forward = self.driver.get_forward()
		rotation = self.driver.get_rotation()

		## Only drive if there's actual input
		if abs(forward) > 0.0 or abs(rotation) > 0.0:
			self.swerve.drive(forward, rotation)
		else:
			self.swerve.stop()

	def disabledInit(self):
		"""Called when robot enters disabled state."""
		if self.swerve is not None:
			self.swerve.stop()


if __name__ == "__main__":
	wpilib.run(Robot)
