"""Single swerve wheel abstraction"""
import wpilib
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, ResetMode, PersistMode


class SwerveWheel:
	"""Represents one swerve module with drive and turn motors"""
	
	def __init__(self, name, drive_canid, turn_canid, encoder_dio, manual_offset):
		self.name = name
		self.drive_motor = SparkMax(drive_canid, SparkLowLevel.MotorType.kBrushless)
		self.turn_motor = SparkMax(turn_canid, SparkLowLevel.MotorType.kBrushless)
		
		# Configure turn motor inversion via SparkMaxConfig (best practice for REV)
		turn_config = SparkMaxConfig()
		turn_config.inverted(True)
		self.turn_motor.configure(turn_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
		
		self.encoder = wpilib.DutyCycleEncoder(encoder_dio)
		self.manual_offset = manual_offset
		self.offset = 0.0
	
	def set_drive_power(self, power):
		"""Set drive motor power (-1.0 to 1.0)"""
		self.drive_motor.set(power)
	
	def set_turn_power(self, power):
		"""Set turn motor power (-1.0 to 1.0)"""
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
