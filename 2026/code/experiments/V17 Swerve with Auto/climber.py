

import wpilib
from rev import SparkMax, SparkFlex, SparkLowLevel
import CANID


class Climber:
	GEAR_RATIO = 1.0  # Adjust based on actual climber gear ratio
	
	# Encoder positions for climber hooks
	TOP_POSITION = 180.0  # Degrees when top hook is at top - EDIT THIS VALUE
	BOTTOM_POSITION = 0.0  # Degrees when bottom hook is at bottom - EDIT THIS VALUE
	
	# Climbing parameters
	NUM_RUNGS = 4  # Number of rungs to climb
	CLIMB_SPEED = 0.3  # Motor speed for climbing
	
	def __init__(self):
		super().__init__()
		
		# Initialize motor controllers
		try:
			self.climber_motor = SparkMax(CANID.CLIMBER, SparkLowLevel.MotorType.kBrushless)
			print(f"[SHOOTER] Climber motor initialized on CAN ID {CANID.CLIMBER}", flush=True)
		except Exception as e:
			print(f"[SHOOTER] ERROR - Failed to initialize climber motor (CAN ID {CANID.CLIMBER}): {type(e).__name__}: {e}", flush=True)
			self.climber_motor = None
		
		# Rev ThruBore v1 encoder on DIO 5
		try:
			self.encoder = wpilib.DutyCycleEncoder(5)
			print(f"[CLIMBER] Encoder initialized on DIO 5", flush=True)
		except Exception as e:
			print(f"[CLIMBER] ERROR - Failed to initialize encoder on DIO 5: {type(e).__name__}: {e}", flush=True)
			self.encoder = None
		
		self.offset = 0.0
		self.current_speed = 0.0
		if self.encoder:
			self.last_encoder_degrees = self.encoder.get() * 360.0
			self.continuous_angle = (self.last_encoder_degrees - self.offset) / self.GEAR_RATIO
		else:
			self.last_encoder_degrees = 0.0
			self.continuous_angle = 0.0
		
		# Climbing state
		self.is_climbing = False
		self.current_rung = 0
		self.climbing_up = True  # True = moving towards top, False = moving towards bottom
		
		
	
	def set_climber(self, speed):
		"""Set climber motor speed (very slow - 5% max recommended)"""
		self.current_speed = speed
		if self.climber_motor:
			self.climber_motor.set(speed)
		#	print("climber speed", speed)
		#else:
		#	print("climber motor does not exist")
	
	def get_raw_encoder_degrees(self):
		"""Get encoder degrees (0-360) - purely from encoder, no offset applied"""
		if self.encoder:
			return self.encoder.get() * 360.0
		return 0.0
	
	def get_raw_angle(self):
		"""Get raw encoder angle in degrees, corrected for gear ratio and offset"""
		if not self.encoder:
			return self.continuous_angle
		
		encoder_degrees = self.get_raw_encoder_degrees()
		
		# Detect encoder wrapping by checking if change > 180 degrees
		encoder_delta = encoder_degrees - self.last_encoder_degrees
		
		# Apply wrap-around correction ONLY for legitimate wraps
		if abs(encoder_delta) > 180:
			if encoder_delta > 180:
				# Wrapped from ~360 down to ~0
				encoder_delta -= 360
			else:
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
		if self.encoder:
			self.continuous_angle = (self.last_encoder_degrees - self.offset) / self.GEAR_RATIO
	
	def get_zero_offset(self):
		"""Get the saved zero offset"""
		return self.offset
	
	def start_climb(self):
		"""Start the autonomous climbing sequence (non-blocking)"""
		if self.current_rung < self.NUM_RUNGS:
			self.is_climbing = True
			self.current_rung = 0
			self.climbing_up = True
			print(f"[CLIMBER] Starting climb sequence - {self.NUM_RUNGS} rungs", flush=True)
	
	def update_climb(self):
		"""Call every cycle to update motor during climbing sequence"""
		if not self.is_climbing:
			return
		
		current_angle = self.get_angle()
		
		if self.climbing_up:
			# Moving towards top position to grab next rung
			if current_angle < self.TOP_POSITION:
				# Move clockwise towards top
				self.set_climber(self.CLIMB_SPEED)
			else:
				# Reached top position, switch direction
				self.climbing_up = False
				self.current_rung += 1
				print(f"[CLIMBER] Reached top - now on rung {self.current_rung}", flush=True)
		else:
			# Moving towards bottom position (descending)
			if current_angle > self.BOTTOM_POSITION:
				# Move counter-clockwise towards bottom
				self.set_climber(-self.CLIMB_SPEED)
			else:
				# Reached bottom position, check if done
				if self.current_rung >= self.NUM_RUNGS:
					# Climb complete
					self.set_climber(0)
					self.is_climbing = False
					print(f"[CLIMBER] Climb sequence complete!", flush=True)
				else:
					# Move back up to next rung
					self.climbing_up = True
					print(f"[CLIMBER] Moving to next rung...", flush=True)
	
	def stop_climb(self):
		"""Stop the climbing sequence and motor"""
		self.is_climbing = False
		self.set_climber(0)
		print(f"[CLIMBER] Climb stopped at rung {self.current_rung}", flush=True)
	