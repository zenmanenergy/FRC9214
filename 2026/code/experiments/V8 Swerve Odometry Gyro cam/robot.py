import wpilib
import wpilib.drive
from wpimath.geometry import Rotation2d

from drive import SwerveDrive, SwerveModule
from joystick_drive import DriverJoystick
from joystick_arm import ArmJoystick
from vision import VisionNetworkTables


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

		# Initialize vision/NetworkTables for AprilTag-based localization
		self.vision = VisionNetworkTables(table_name="vision")
		
		# Set vision trust (0.8 = 80% trust vision when available, 20% trust odometry)
		self.swerve.set_vision_trust(0.8)

		# Drive mode flags
		self.slow_mode = False

	def teleopPeriodic(self):
		"""Handle teleop control inputs."""
		# Update odometry with encoder and gyro data
		self.swerve.update_odometry()
		
		# Check for vision updates from coprocessor
		vision_pose, has_vision = self.vision.get_camera_pose()
		if has_vision:
			# Update odometry with vision data when available
			self.swerve.update_vision_pose(vision_pose, vision_confidence=0.9)
		
		# Publish our odometry back to coprocessor for reference
		self.vision.publish_odometry(self.swerve.get_pose(), confidence=0.85)
		
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
		self._print_debug_info(has_vision)

	def disabledInit(self):
		"""Called when robot enters disabled state."""
		self.swerve.stop()

	def disabledPeriodic(self):
		"""Called periodically while robot is disabled."""
		pass

	# ========================================================================
	# UTILITY FUNCTIONS
	# ========================================================================

	def _print_debug_info(self, has_vision: bool = False):
		"""Print debug information to console."""
		angles = self.swerve.get_module_angles()
		pose = self.swerve.get_pose()
		
		wpilib.SmartDashboard.putNumber("FL Angle", angles[0].degrees())
		wpilib.SmartDashboard.putNumber("FR Angle", angles[1].degrees())
		wpilib.SmartDashboard.putNumber("RL Angle", angles[2].degrees())
		wpilib.SmartDashboard.putNumber("RR Angle", angles[3].degrees())
		
		# Odometry/Vision data
		wpilib.SmartDashboard.putNumber("Robot X", pose.X())
		wpilib.SmartDashboard.putNumber("Robot Y", pose.Y())
		wpilib.SmartDashboard.putNumber("Robot Heading", pose.rotation().degrees())
		wpilib.SmartDashboard.putBoolean("Vision Available", has_vision)
		wpilib.SmartDashboard.putBoolean("Slow Mode", self.slow_mode)


# ==============================================================================
# ENTRY POINT
# ==============================================================================

if __name__ == "__main__":
	wpilib.run(Robot)
