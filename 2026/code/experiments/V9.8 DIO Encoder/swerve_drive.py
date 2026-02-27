"""Swerve drive controller for all 4 wheels"""
import wpilib
from wheel import SwerveWheel
from encoder_calibration import EncoderCalibration
import swerve_config as config


class SwerveDrive:
	"""Main swerve drive controller"""
	
	def __init__(self):
		print("[ROBOT] Initializing swerve drive with 4 wheels...")
		
		# Create all wheels
		self.wheels = {}
		for wheel_name, pin_config in config.WHEELS.items():
			self.wheels[wheel_name] = SwerveWheel(
				wheel_name,
				pin_config["drive_canid"],
				pin_config["turn_canid"],
				pin_config["encoder_dio"],
				config.MANUAL_OFFSETS[wheel_name]
			)
		
		# Load calibration
		self.calibration = EncoderCalibration()
		for wheel_name, offset in self.calibration.offsets.items():
			self.wheels[wheel_name].offset = offset
		
		print("[ROBOT] Initialized 4 wheels with offsets")
		
		# Alignment state
		self.aligning = False
		self.align_start_time = None
	
	def stop_all(self):
		"""Stop all motors"""
		for wheel in self.wheels.values():
			wheel.stop()
	
	def set_wheel_drive_power(self, wheel_name, power):
		"""Set drive power for specific wheel"""
		if wheel_name in self.wheels:
			self.wheels[wheel_name].set_drive_power(power)
	
	def set_wheel_turn_power(self, wheel_name, power):
		"""Set turn power for specific wheel"""
		if wheel_name in self.wheels:
			self.wheels[wheel_name].set_turn_power(power)
	
	def get_wheel_angle(self, wheel_name):
		"""Get angle of specific wheel"""
		if wheel_name in self.wheels:
			return self.wheels[wheel_name].get_angle()
		return -1
	
	def set_wheel_zero(self, wheel_name):
		"""Save current position as zero for a wheel"""
		if wheel_name in self.wheels:
			wheel = self.wheels[wheel_name]
			wheel.set_zero_offset(wheel.get_raw_angle())
			self.calibration.set_offset(wheel_name, wheel.offset)
			self.calibration.save_offsets()
			print(f"[ZEROING] Saved offset for {wheel_name}: {wheel.offset:.1f}")
	
	def start_alignment(self):
		"""Start aligning all wheels to 0 degrees"""
		self.aligning = True
		self.align_start_time = wpilib.Timer.getFPGATimestamp()
		print("[ALIGN] Starting alignment to 0 degrees for all wheels...")
		for wheel_name in self.wheels.keys():
			print(f"  {wheel_name}: moving to 0")
	
	def update_alignment(self):
		"""Update alignment routine (call every loop)"""
		if not self.aligning:
			return
		
		elapsed = wpilib.Timer.getFPGATimestamp() - self.align_start_time
		
		if elapsed < config.ALIGN_TIMEOUT:
			all_aligned = True
			for wheel_name, wheel in self.wheels.items():
				current_angle = wheel.get_angle()
				
				# Normalize to -180 to 180
				if current_angle > 180:
					current_angle -= 360
				
				# P controller
				error = -current_angle  # Error to reach 0
				if abs(error) > config.ALIGN_TOLERANCE:
					all_aligned = False
					speed = max(-config.MOTOR_SCALE_ALIGN, 
							   min(config.MOTOR_SCALE_ALIGN, error * config.ALIGN_KP))
				wheel.set_turn_power(speed)
				print("[ALIGN] All wheels aligned to 0!")
				self.aligning = False
		else:
			print("[ALIGN] Alignment timeout!")
			self.stop_all()
			self.aligning = False
