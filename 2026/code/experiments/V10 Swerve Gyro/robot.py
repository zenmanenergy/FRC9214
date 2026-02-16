import wpilib
import wpilib.drive
from wpimath.geometry import Rotation2d

from drive import SwerveDrive, SwerveModule
from joystick_drive import DriverJoystick
from joystick_arm import ArmJoystick
from gyro import NavXGyro


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

		# Initialize NavX gyro for field-relative drive
		self.gyro = NavXGyro()

		# Drive mode flags
		self.slow_mode = False
		self.field_relative = False

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

		# Toggle field-relative drive
		if self.driver.get_field_relative_button_pressed():
			self.field_relative = not self.field_relative

		# Reset gyro
		if self.driver.get_reset_gyro_button_pressed():
			self._reset_gyro()

		# Stop button
		if self.driver.get_stop_button():
			self.swerve.stop()
		else:
			self.swerve.drive(forward, strafe, rotation, field_relative=self.field_relative)

		# Update robot angle from gyro
		self._update_robot_angle()

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

	def _reset_gyro(self):
		"""Reset gyro angle to 0 degrees."""
		if self.gyro.is_ready():
			self.gyro.reset()
			wpilib.SmartDashboard.putString("Gyro Status", "Reset")

	def _update_robot_angle(self):
		"""Update swerve drive with current gyro angle."""
		if self.gyro.is_ready():
			angle = self.gyro.get_angle()
			self.swerve.set_robot_angle(angle)

	def _print_debug_info(self):
		"""Print debug information to console."""
		angles = self.swerve.get_module_angles()
		wpilib.SmartDashboard.putNumber("FL Angle", angles[0].degrees())
		wpilib.SmartDashboard.putNumber("FR Angle", angles[1].degrees())
		wpilib.SmartDashboard.putNumber("RL Angle", angles[2].degrees())
		wpilib.SmartDashboard.putNumber("RR Angle", angles[3].degrees())
		wpilib.SmartDashboard.putBoolean("Slow Mode", self.slow_mode)
		wpilib.SmartDashboard.putBoolean("Field Relative", self.field_relative)

		if self.gyro.is_ready():
			wpilib.SmartDashboard.putNumber("Robot Angle", self.gyro.get_angle_degrees())
			wpilib.SmartDashboard.putNumber("Angular Velocity", self.gyro.get_angular_velocity())
			wpilib.SmartDashboard.putString("Gyro Status", "Connected")
		else:
			wpilib.SmartDashboard.putString("Gyro Status", "Disconnected")


# ==============================================================================
# ENTRY POINT
# ==============================================================================

if __name__ == "__main__":
	wpilib.run(Robot)
