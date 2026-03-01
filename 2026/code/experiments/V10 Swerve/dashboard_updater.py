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
