"""Single swerve wheel abstraction"""
import wpilib
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, ResetMode, PersistMode


class SwerveWheel:
	"""Represents one swerve module with drive and turn motors"""
	
	def __init__(self, name, drive_canid, turn_canid, encoder_dio, manual_offset):
		self.name = name
		self.drive_motor = SparkMax(drive_canid, SparkLowLevel.MotorType.kBrushless)
		self.turn_motor = SparkMax(turn_canid, SparkLowLevel.MotorType.kBrushless)
		
		# Direction is handled via speed flip in drive_wheel_to_angle
		
		self.encoder = wpilib.DutyCycleEncoder(encoder_dio)
		self.manual_offset = manual_offset
		self.offset = 0.0
		self.current_drive_power = 0.0  # Track current drive power for dashboard
	
	def set_drive_power(self, power):
		"""Set drive motor power (-1.0 to 1.0)"""
		self.current_drive_power = power
		self.drive_motor.set(power)
	
	def set_turn_power(self, power):
		"""Set turn motor power (-1.0 to 1.0)"""
		print(f"[WHEEL-MOTOR] {self.name}: set_turn_power({power:.4f})")
		self.turn_motor.set(power)
	
	def stop(self):
		"""Stop both motors"""
		self.drive_motor.set(0.0)
		self.turn_motor.set(0.0)
	
	def get_raw_angle(self):
		"""Get raw encoder angle in degrees (0-360)"""
		return self.encoder.get() * 360.0
	
	def get_angle(self):
		"""Get adjusted angle accounting for offset (0-360)"""
		raw = self.get_raw_angle()
		angle = (raw - self.offset) % 360.0
		return int(round(angle)) % 360
	
	def set_zero_offset(self, raw_angle):
		"""Save this raw angle as zero point"""
		self.offset = raw_angle
	
	def get_zero_offset(self):
		"""Get the saved zero offset"""
		return self.offset
	
	def get_drive_power(self):
		"""Get current drive motor power (-1.0 to 1.0)"""
		return self.current_drive_power
