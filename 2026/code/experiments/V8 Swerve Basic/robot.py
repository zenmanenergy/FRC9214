import wpilib
import wpilib.drive
from wpimath.geometry import Rotation2d

from drive import SwerveDrive, SwerveModule
from joystick_drive import DriverJoystick
from joystick_arm import ArmJoystick


# ==============================================================================
# MAIN ROBOT CLASS
# ==============================================================================


class Robot(wpilib.TimedRobot):
	"""Main robot container and teleop code."""

	def robotInit(self):
		"""Initialize robot and subsystems."""
		# Initialize joysticks
		self.driver = DriverJoystick()
		self.operator = ArmJoystick()

		# Create swerve modules (adjust CAN IDs and encoder channels as needed)
		self.front_left = SwerveModule(drive_motor_id=1, turn_motor_id=5, absolute_encoder_channel=0)
		self.front_right = SwerveModule(drive_motor_id=2, turn_motor_id=6, absolute_encoder_channel=1)
		self.rear_left = SwerveModule(drive_motor_id=3, turn_motor_id=7, absolute_encoder_channel=2)
		self.rear_right = SwerveModule(drive_motor_id=4, turn_motor_id=8, absolute_encoder_channel=3)

		# Create swerve drive
		self.swerve = SwerveDrive(
			self.front_left,
			self.front_right,
			self.rear_left,
			self.rear_right,
			track_width_m=0.5,  # Adjust to your robot
			wheelbase_m=0.5,  # Adjust to your robot
		)

		# Optional: Initialize gyro for field-relative drive
		# self.gyro = wpilib.ADIS16448_IMU()

		# Drive mode flags
		self.slow_mode = False

	def teleopPeriodic(self):
		"""Handle teleop control inputs."""
		# Get joystick inputs (deadband already applied)
		forward = self.driver.get_forward()
		strafe = self.driver.get_strafe()
		rotation = self.driver.get_rotation()

		# Apply slow mode multiplier
		if self.driver.get_slow_mode_button():
			self.slow_mode = True
		else:
			self.slow_mode = False

		if self.slow_mode:
			forward *= 0.3
			strafe *= 0.3
			rotation *= 0.3

		# Stop button
		if self.driver.get_stop_button():
			self.swerve.stop()
		else:
			self.swerve.drive(forward, strafe, rotation, field_relative=False)

		# Optional: Print debug info
		self._print_debug_info()

	def disabledInit(self):
		"""Called when robot enters disabled state."""
		self.swerve.stop()

	def disabledPeriodic(self):
		"""Called periodically while robot is disabled."""
		pass

	# ========================================================================
	# UTILITY FUNCTIONS
	# ========================================================================

	def _print_debug_info(self):
		"""Print debug information to console."""
		angles = self.swerve.get_module_angles()
		wpilib.SmartDashboard.putNumber("FL Angle", angles[0].degrees())
		wpilib.SmartDashboard.putNumber("FR Angle", angles[1].degrees())
		wpilib.SmartDashboard.putNumber("RL Angle", angles[2].degrees())
		wpilib.SmartDashboard.putNumber("RR Angle", angles[3].degrees())
		wpilib.SmartDashboard.putBoolean("Slow Mode", self.slow_mode)


# ==============================================================================
# ENTRY POINT
# ==============================================================================

if __name__ == "__main__":
	wpilib.run(Robot)
