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
		self.target_align_angle = 0  # Target angle for alignment
	
	def stop_all(self):
		"""Stop all motors"""
		for wheel in self.wheels.values():
			wheel.stop()
	
	def rotate_to_angle(self, angle):
		"""Rotate all wheels to a specific angle (0-360)"""
		if not self.aligning:
			self.start_alignment(angle)
	
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
			print(f"[ZEROING] Saved offset for {wheel_name}: {wheel.offset:.1f} (0 degrees)")
	
	def set_wheel_angle(self, wheel_name, target_angle):
		"""Save current position as a specific angle (0, 90, 180, 270)"""
		if wheel_name in self.wheels:
			wheel = self.wheels[wheel_name]
			raw = wheel.get_raw_angle()
			# Calculate offset: offset = raw - target
			calculated_offset = (raw - target_angle) % 360
			
			print(f"\n[SETANGLE] === Calibrating {wheel_name} to {target_angle}° ===")
			print(f"[SETANGLE] Step 1: Read encoder")
			print(f"           raw = {raw:.1f}°")
			print(f"[SETANGLE] Step 2: Calculate offset")
			print(f"           offset = ({raw:.1f} - {target_angle}) % 360 = {calculated_offset:.1f}°")
			
			wheel.offset = calculated_offset
			self.calibration.set_offset(wheel_name, calculated_offset)
			print(f"[SETANGLE] Step 3: Save to file")
			self.calibration.save_offsets()
			
			print(f"[SETANGLE] Step 4: Reload all offsets from file")
			for name, offset in self.calibration.offsets.items():
				self.wheels[name].offset = offset
				print(f"           {name}: offset={offset:.1f}°")
			
			print(f"[SETANGLE] Step 5: Verify calculation")
			verify_angle = (raw - wheel.offset) % 360
			print(f"           ({raw:.1f} - {wheel.offset:.1f}) % 360 = {verify_angle:.1f}°")
			print(f"[SETANGLE] === Complete ===\n")
	
	def start_alignment(self, target_angle=0):
		"""Start aligning all wheels to target angle"""
		self.aligning = True
		self.align_start_time = wpilib.Timer.getFPGATimestamp()
		self.target_align_angle = target_angle
		print(f"[ALIGN] Starting alignment to {target_angle}° for all wheels...")
		for wheel_name in self.wheels.keys():
			print(f"  {wheel_name}: moving to {target_angle}°")
	
	def update_alignment(self):
		"""Update alignment routine (call every loop)"""
		if not self.aligning:
			return
		
		elapsed = wpilib.Timer.getFPGATimestamp() - self.align_start_time
		
		if elapsed < config.ALIGN_TIMEOUT:
			all_aligned = True
			for wheel_name, wheel in self.wheels.items():
				current_angle = wheel.get_angle()
				
				# Calculate shortest path to target
				error = self.target_align_angle - current_angle
				# Normalize to -180 to 180
				if error > 180:
					error -= 360
				elif error < -180:
					error += 360
				
				# P controller
				if abs(error) > config.ALIGN_TOLERANCE:
					all_aligned = False
					speed = max(-config.MOTOR_SCALE_ALIGN, 
							   min(config.MOTOR_SCALE_ALIGN, error * config.ALIGN_KP))
					wheel.set_turn_power(speed)
				else:
					wheel.set_turn_power(0)
			
			if all_aligned:
				print(f"[ALIGN] All wheels aligned to {self.target_align_angle}°!")
				self.aligning = False
		else:
			print("[ALIGN] Alignment timeout!")
			self.stop_all()
			self.aligning = False
