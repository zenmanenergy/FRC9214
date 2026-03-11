import wpilib
from rev import SparkMax, SparkLowLevel
import CANID


class ShooterSubsystem:
	def __init__(self, turret=None):
		super().__init__()
		
		# Initialize motor controllers
		self.uptake_motor = SparkMax(CANID.SHOOTER_UPTAKE, SparkLowLevel.MotorType.kBrushless)
		self.shooter_motor = SparkMax(CANID.SHOOTER_SHOOTER, SparkLowLevel.MotorType.kBrushless)
		
		# Turret reference (optional)
		self.turret = turret
		
		# Turret rotation state
		self.target_turret_angle = None
		self.rotating_to_angle = False
		
		# Invert uptake motor if needed
		self.uptake_motor.setInverted(False)
	
	def set_uptake(self, speed):
		"""Set uptake motor speed"""
		self.uptake_motor.set(speed)
	
	def set_turret(self, speed):
		"""
		Set turret motor speed, respecting rotation limits.
		
		Args:
			speed: Motor power (-1.0 to 1.0)
				Negative = rotate left
				Positive = rotate right
		"""
		if not self.turret:
			return
		
		# Get current turret angle and limits
		current_angle = self.turret.get_angle()
		left_limit = self.turret.get_left_limit()
		right_limit = self.turret.get_right_limit()
		
		# Check if we're trying to rotate beyond limits
		if speed < 0:
			# Trying to rotate left
			if current_angle <= left_limit:
				# At or past left limit, stop
				self.turret.set_turn_power(0)
				return
		elif speed > 0:
			# Trying to rotate right
			if current_angle >= right_limit:
				# At or past right limit, stop
				self.turret.set_turn_power(0)
				return
		
		# Apply the requested speed
		self.turret.set_turn_power(speed)
	
	def rotate_to_angle(self, target_angle):
		"""
		Rotate turret to target angle, respecting rotation limits.
		Call update_rotation() every cycle to continuously adjust.
		
		Args:
			target_angle: Target angle in degrees (0-360)
		"""
		if not self.turret:
			return
		
		# Check if target angle is within limits
		left_limit = self.turret.get_left_limit()
		right_limit = self.turret.get_right_limit()
		
		# Normalize target angle for comparison
		norm_target = target_angle % 360
		
		# Check limit bounds
		if norm_target < left_limit or norm_target > right_limit:
			print(f"[TURRET] Target angle {norm_target}° is outside limits ({left_limit}° to {right_limit}°). Stopping.")
			self.turret.stop()
			self.rotating_to_angle = False
			self.target_turret_angle = None
			return
		
		# Valid target - start rotation
		self.target_turret_angle = norm_target
		self.rotating_to_angle = True
		print(f"[TURRET] Starting rotation to {norm_target}°")
	
	def update_rotation(self):
		"""
		Update turret rotation towards target angle.
		Call this every cycle when rotating.
		"""
		if not self.rotating_to_angle or not self.target_turret_angle or not self.turret:
			return
		
		current_angle = self.turret.get_angle()
		target_angle = self.target_turret_angle
		
		# Calculate shortest error path
		error = target_angle - current_angle
		if error > 180:
			error -= 360
		elif error < -180:
			error += 360
		
		# Check if we've reached target (tolerance of 2 degrees)
		if abs(error) < 2.0:
			print(f"[TURRET] Reached target angle {target_angle}°")
			self.turret.stop()
			self.rotating_to_angle = False
			self.target_turret_angle = None
			return
		
		# Proportional control: apply power proportional to error
		# KP = 0.004 gives reasonable speed for ~180° turns
		kp = 0.004
		motor_power = max(-0.3, min(0.3, error * kp))  # Clamp to ±30%
		
		self.turret.set_turn_power(motor_power)
	
	def set_shooter(self, speed):
		"""Set shooter motor speed"""
		self.shooter_motor.set(speed)
	
	def stop_all(self):
		"""Stop all motors"""
		self.uptake_motor.set(0)
		self.shooter_motor.set(0)
		if self.turret:
			self.turret.stop()
		self.rotating_to_angle = False
		self.target_turret_angle = None
