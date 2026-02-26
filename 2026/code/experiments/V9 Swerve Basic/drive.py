import math
try:
	import rev
	HAS_REV = True
except (ImportError, ModuleNotFoundError):
	HAS_REV = False
	rev = None


class SwerveModule:
	"""A single swerve module with drive and turn motors."""

	def __init__(self, drive_motor_id: int, turn_motor_id: int):
		"""Initialize a swerve module with REV SparkMax motors."""
		self.drive_motor_id = drive_motor_id
		self.turn_motor_id = turn_motor_id
		self.drive_motor = None
		self.turn_motor = None
		
		if not HAS_REV:
			return

		self.drive_motor = rev.SparkMax(drive_motor_id, rev.SparkMax.MotorType.kBrushless)
		self.turn_motor = rev.SparkMax(turn_motor_id, rev.SparkMax.MotorType.kBrushless)
		
		print("M%d INIT: Drive=%d Turn=%d" % (drive_motor_id, drive_motor_id, turn_motor_id))

	def set_drive(self, speed: float):
		"""Set drive motor speed."""
		if self.drive_motor is None:
			return
		self.drive_motor.set(speed)

	def set_turn(self, speed: float):
		"""Set turn motor speed."""
		if self.turn_motor is None:
			return
		self.turn_motor.set(speed)

	def stop(self):
		"""Stop both motors."""
		if self.drive_motor is not None:
			self.drive_motor.set(0.0)
		if self.turn_motor is not None:
			self.turn_motor.set(0.0)


class SwerveDrive:
	"""Controls a 4-wheel swerve drive."""

	def __init__(
		self,
		front_left: SwerveModule,
		front_right: SwerveModule,
		rear_left: SwerveModule,
		rear_right: SwerveModule,
	):
		"""Initialize the swerve drive."""
		self.front_left = front_left
		self.front_right = front_right
		self.rear_left = rear_left
		self.rear_right = rear_right

	def drive(self, forward_speed: float, rotation_speed: float):
		"""Drive with forward/backward and rotation."""
		# All drive motors get forward speed
		self.front_left.set_drive(forward_speed*.3)
		self.front_right.set_drive(forward_speed*.3)
		self.rear_left.set_drive(forward_speed*.3)
		self.rear_right.set_drive(forward_speed*.3)
		
		# All turn motors get rotation speed (negated to reverse direction)
		self.front_left.set_turn(-rotation_speed*.3)
		self.front_right.set_turn(-rotation_speed*.3)
		self.rear_left.set_turn(-rotation_speed*.3)
		self.rear_right.set_turn(-rotation_speed*.3)

	def stop(self):
		"""Stop all motors."""
		self.front_left.stop()
		self.front_right.stop()
		self.rear_left.stop()
		self.rear_right.stop()
