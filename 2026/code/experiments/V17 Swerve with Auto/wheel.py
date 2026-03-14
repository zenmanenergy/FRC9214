"""Single swerve wheel abstraction"""
import wpilib
from rev import SparkMax, SparkLowLevel


class SwerveWheel:
	"""Represents one swerve module with drive and turn motors"""
	
	def __init__(self, name, drive_canid, turn_canid, encoder_dio, manual_offset):
		self.name = name
		
		# Initialize motors with error handling
		try:
			self.drive_motor = SparkMax(drive_canid, SparkLowLevel.MotorType.kBrushless)
			if name == "rear_right":
				print(f"[WHEEL] {name} drive motor initialized on CAN {drive_canid}", flush=True)
		except Exception as e:
			print(f"[WHEEL] ERROR - Failed to initialize drive motor for {name} (CAN ID {drive_canid}): {type(e).__name__}: {e}", flush=True)
			self.drive_motor = None
		
		try:
			self.turn_motor = SparkMax(turn_canid, SparkLowLevel.MotorType.kBrushless)
			if name == "rear_right":
				print(f"[WHEEL] {name} turn motor initialized on CAN {turn_canid}", flush=True)
		except Exception as e:
			print(f"[WHEEL] ERROR - Failed to initialize turn motor for {name} (CAN ID {turn_canid}): {type(e).__name__}: {e}", flush=True)
			self.turn_motor = None
		
		# Direction is handled via speed flip in drive_wheel_to_angle
		
		self.encoder = wpilib.DutyCycleEncoder(encoder_dio)
		self.manual_offset = manual_offset
		self.offset = 0.0
		self.current_drive_power = 0.0  # Track current drive power for dashboard
		self.manual_offset = manual_offset
		self.offset = 0.0
		self.current_drive_power = 0.0  # Track current drive power for dashboard
	
	def set_drive_power(self, power):
		"""Set drive motor power (-1.0 to 1.0)"""
		self.current_drive_power = power
		if self.drive_motor:
			# Negate power for front_right TEST (wires appear to be physically reversed)
			motor_power = power
			if self.name == "front_right":
				motor_power = -power
			
			self.drive_motor.set(motor_power)
			# Debug for front_right
			if self.name == "front_right" and abs(power) > 0.9:
				# Only print when power is high (to avoid spam)
				try:
					actual_power = self.drive_motor.get()
					if actual_power != motor_power:
						print(f"[FR-DRIVE-CMD] Set power to {motor_power:.3f} but motor.get() returned {actual_power:.3f}", flush=True)
				except:
					pass  # Ignore errors in reading back power
		else:
			print(f"[DRIVE_MOTOR] {self.name} drive motor is NULL - cannot send command", flush=True)
	
	def set_turn_power(self, power):
		"""Set turn motor power (-1.0 to 1.0)"""
		if self.turn_motor:
			# Invert power for all wheels (they're all mounted the same way physically)
			if self.name in ["front_right", "front_left", "rear_right", "rear_left"]:
				power = -power
			self.turn_motor.set(power)
		else:
			print(f"[MOTOR] {self.name} turn motor is NULL - cannot send command", flush=True)
	
	def stop(self):
		"""Stop both motors"""
		if self.drive_motor:
			self.drive_motor.set(0.0)
		if self.turn_motor:
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
	
	def get_drive_position(self):
		"""Get drive motor encoder position in rotations"""
		if self.drive_motor:
			try:
				return self.drive_motor.getEncoder().getPosition()
			except:
				return 0.0
		return 0.0
	
	def get_drive_velocity(self):
		"""Get drive motor encoder velocity in RPM"""
		if self.drive_motor:
			try:
				return self.drive_motor.getEncoder().getVelocity()
			except:
				return 0.0
		return 0.0
	
	def get_drive_distance(self, wheel_diameter_inches=4.0):
		"""Get total distance traveled in inches"""
		position = self.get_drive_position()
		# Convert rotations to distance: distance = rotations * pi * diameter
		import math
		# Account for front_right motor inversion in set_drive_power
		if self.name == "front_right":
			position = -position
		return position * math.pi * wheel_diameter_inches
