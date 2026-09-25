from wpilib import SmartDashboard


class DashboardUpdater:
	"""Manages all SmartDashboard updates for the swerve drive."""

	def __init__(self, swerve_drive):
		"""
		Initialize the dashboard updater.

		Args:
			swerve_drive: SwerveDrive instance to read wheel angles from
		"""
		self.drive = swerve_drive
		self.counter = 0

	def update(self):
		"""Update all dashboard values. Call this from robotPeriodic()."""
		# Test counter
		SmartDashboard.putNumber("Counter", self.counter)
		self.counter += 1

		# Wheel angles (0-360 degrees) - using v9.9 key names
		SmartDashboard.putNumber("FR Angle", self.drive.get_wheel_angle("front_right"))
		SmartDashboard.putNumber("RR Angle", self.drive.get_wheel_angle("rear_right"))
		SmartDashboard.putNumber("RL Angle", self.drive.get_wheel_angle("rear_left"))
		SmartDashboard.putNumber("FL Angle", self.drive.get_wheel_angle("front_left"))
		
		# Wheel drive power (-1.0 to 1.0)
		SmartDashboard.putNumber("FR Power", self.drive.get_wheel_power("front_right"))
		SmartDashboard.putNumber("RR Power", self.drive.get_wheel_power("rear_right"))
		SmartDashboard.putNumber("RL Power", self.drive.get_wheel_power("rear_left"))
		SmartDashboard.putNumber("FL Power", self.drive.get_wheel_power("front_left"))
		
		# Odometry - distance traveled (metric only)
		SmartDashboard.putNumber("Distance Centimeters", self.drive.odometry.get_total_distance())
		SmartDashboard.putNumber("Distance Meters", self.drive.odometry.get_distance_meters())

		# Odometry - pose
		x, y = self.drive.odometry.get_position()
		SmartDashboard.putNumber("Odometry X", round(x, 1))
		SmartDashboard.putNumber("Odometry Y", round(y, 1))
		SmartDashboard.putNumber("Odometry Heading", round(self.drive.odometry.get_heading(), 1))
		
		# IMU diagnostics
		SmartDashboard.putBoolean("IMU Connected", self.drive.imu.is_connected())
		SmartDashboard.putBoolean("IMU Calibrating", self.drive.imu.is_calibrating())
		SmartDashboard.putBoolean("IMU Inverted", self.drive.imu.invert)
		SmartDashboard.putNumber("IMU Raw Yaw", round(self.drive.imu.ahrs.getYaw(), 1))
		SmartDashboard.putNumber("IMU Heading (0-360)", round(self.drive.imu.get_heading(), 1))
		SmartDashboard.putNumber("IMU Pitch", round(self.drive.imu.get_pitch(), 1))
		SmartDashboard.putNumber("IMU Roll", round(self.drive.imu.get_roll(), 1))
		
		# Odometry vs IMU heading comparison
		SmartDashboard.putNumber("Odometry Heading Delta", round(self.drive.odometry.get_last_heading_delta(), 1))
		wheel_heading = self.drive.odometry._heading
		imu_heading = self.drive.imu.get_heading()
		heading_diff = (imu_heading - wheel_heading) % 360
		if heading_diff > 180:
			heading_diff -= 360
		SmartDashboard.putNumber("IMU vs Wheel Heading Diff", round(heading_diff, 1))
