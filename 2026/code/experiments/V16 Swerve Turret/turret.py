"""Turret rotation controller with Rev ThruBore encoder v1"""
import wpilib
from rev import SparkMax, SparkLowLevel


class Turret:
	"""Represents the turret with rotation motor and absolute encoder"""
	
	# Gear ratio: outer gear 260mm / encoder gear 60mm = 4.333:1
	# Encoder spins 4.333 times per turret rotation
	GEAR_RATIO = 260.0 / 60.0  # = 4.333...
	
	def __init__(self, turn_canid, encoder_dio, manual_offset=0.0):
		self.name = "turret"
		self.turn_motor = SparkMax(turn_canid, SparkLowLevel.MotorType.kBrushless)
		
		# Rev ThruBore v1 encoder on DIO
		self.encoder = wpilib.DutyCycleEncoder(encoder_dio)
		self.manual_offset = manual_offset
		self.offset = 0.0
		self.current_turn_power = 0.0  # Track current turn power for dashboard
		self.last_encoder_degrees = self.encoder.get() * 360.0  # Initialize to current encoder position
		# Initialize continuous angle based on actual encoder position
		self.continuous_angle = (self.last_encoder_degrees - self.offset) / self.GEAR_RATIO
	
	def set_turn_power(self, power):
		"""Set turn motor power (-1.0 to 1.0)"""
		self.current_turn_power = power
		self.turn_motor.set(power)
	
	def stop(self):
		"""Stop the motor"""
		self.turn_motor.set(0.0)
	
	def get_raw_encoder_degrees(self):
		"""Get encoder degrees (0-360) - purely from encoder, no offset applied"""
		return self.encoder.get() * 360.0
	
	def get_raw_angle(self):
		"""Get raw encoder angle in degrees, corrected for gear ratio and offset"""
		encoder_degrees = self.get_raw_encoder_degrees()
		
		# Detect encoder wrapping by checking if change > 180 degrees
		encoder_delta = encoder_degrees - self.last_encoder_degrees
		if encoder_delta > 180:
			# Wrapped from ~360 down to ~0
			encoder_delta -= 360
		elif encoder_delta < -180:
			# Wrapped from ~0 up to ~360
			encoder_delta += 360
		
		# Update continuous angle tracking
		self.continuous_angle += encoder_delta / self.GEAR_RATIO
		self.last_encoder_degrees = encoder_degrees
		
		return self.continuous_angle
	
	def get_angle(self):
		"""Get adjusted angle accounting for offset (0-360)"""
		raw = self.get_raw_angle()
		return int(round(raw)) % 360
	
	def set_zero_offset(self, raw_angle):
		"""Save this raw angle as zero point"""
		self.offset = raw_angle
		# Reinitialize continuous angle based on new offset
		self.continuous_angle = (self.last_encoder_degrees - self.offset) / self.GEAR_RATIO
	
	def get_zero_offset(self):
		"""Get the saved zero offset"""
		return self.offset
	
	def set_left_limit(self, angle):
		"""Save this turret angle (0-360) as left rotation limit"""
		self.left_limit = angle
	
	def get_left_limit(self):
		"""Get the saved left limit as turret angle (0-360)"""
		return int(getattr(self, 'left_limit', 0))
	
	def set_right_limit(self, angle):
		"""Save this turret angle (0-360) as right rotation limit"""
		self.right_limit = angle
	
	def get_right_limit(self):
		"""Get the saved right limit as turret angle (0-360)"""
		return int(getattr(self, 'right_limit', 360))
	
	def get_turn_power(self):
		"""Get current turn motor power (-1.0 to 1.0)"""
		return self.current_turn_power
