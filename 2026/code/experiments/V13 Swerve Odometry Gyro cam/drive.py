"""
Example usage of the swerve drive library.

For the actual library classes, import from the swerve package:
    from swerve import SwerveDrive, SwerveModule, NavXGyro
"""

from swerve import SwerveDrive, SwerveModule, NavXGyro
from wpimath.geometry import Pose2d

# Example initialization (adjust motor/encoder IDs for your robot):
def create_swerve_drive() -> SwerveDrive:
	"""Create and initialize a swerve drive system."""
	
	# Create gyro
	gyro = NavXGyro()
	
	# Create swerve modules (drive_id, turn_id, abs_encoder_channel, wheel_encoder_channel)
	front_left = SwerveModule(1, 2, 0, 8)
	front_right = SwerveModule(3, 4, 1, 9)
	rear_left = SwerveModule(5, 6, 2, 10)
	rear_right = SwerveModule(7, 8, 3, 11)
	
	# Create swerve drive system
	drive = SwerveDrive(
		front_left=front_left,
		front_right=front_right,
		rear_left=rear_left,
		rear_right=rear_right,
		track_width_m=0.6,
		wheelbase_m=0.6,
		gyro=gyro,
	)
	
	return drive


if __name__ == "__main__":
	# Example usage
	drive = create_swerve_drive()
	
	# Calibrate gyro on startup
	if drive.is_gyro_ready():
		drive.calibrate_gyro()
	
	# Reset odometry to origin
	drive.reset_odometry(Pose2d())
	
	# Drive forward
	drive.drive(forward_speed=0.5, strafe_speed=0.0, rotation_speed=0.0, field_relative=False)
	
	# Update odometry (call this regularly in your robot loop)
	drive.update_odometry()
	
	# Get current position
	pose = drive.get_pose()
	print(f"Position: ({pose.X()}, {pose.Y()}) Angle: {pose.rotation().degrees()}°")

