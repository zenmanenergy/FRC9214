"""Swerve drive controller for all 4 wheels"""
import wpilib
from wpilib import SmartDashboard, RobotController
from .swerve_wheel import SwerveWheel
from .encoder_calibration import EncoderCalibration
from .pid_controller import PIDController
from .swerve_odometry import SwerveOdometry
from . import swerve_config as config


class SwerveDrive:
	"""Main swerve drive controller"""
	
	def __init__(self):
		
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
			# Apply offsets to all wheels
			if wheel_name in self.wheels:
				self.wheels[wheel_name].offset = offset
		
		# Load alignment gains from calibration (required for wheel rotation)
		self.alignment_gains = self.calibration.get_alignment_gains()
		
		# Alignment state
		self.aligning = False
		self.align_start_time = None
		self.target_align_angle = 0  # Target angle for alignment
		self.align_debug_counter = 0  # Counter for debug output
		
		# Load PID gains from calibration file with voltage interpolation
		battery_voltage = RobotController.getBatteryVoltage()
		pid_gains = self.calibration.get_interpolated_gains(battery_voltage)
		
		import sys
		
		# Create PID controllers for each wheel with per-wheel gains if available
		self.pid_controllers = {}
		for wheel_name in self.wheels.keys():
			# Check if we have per-wheel gains
			if isinstance(pid_gains, dict) and wheel_name in pid_gains:
				kp = pid_gains[wheel_name]["kp"]
				ki = pid_gains[wheel_name]["ki"]
				kd = pid_gains[wheel_name]["kd"]
			else:
				# Fallback to uniform gains
				kp = pid_gains.get("kp", 0.003)
				ki = pid_gains.get("ki", 0.005)
				kd = pid_gains.get("kd", 0.0001)
			
			self.pid_controllers[wheel_name] = PIDController(
				kp=kp,
				ki=ki,
				kd=kd,
				name=f"Wheel_{wheel_name}"
			)
		
		# Initialize odometry tracking for distance calculation
		self.odometry = SwerveOdometry(self.wheels, wheel_diameter_cm=10.16)
		
		# Publish tuning history to NetworkTables on startup (for remote dashboard)
		self._publish_tuning_history_to_nt()
		
		# Autotune state
		self.autotuning = False
		self.autotune_wheel = None
		self.autotune_gains = None
		self.autotune_start_time = None
		
		# Per-wheel alignment state (dict so multiple wheels can align simultaneously)
		self.wheel_alignment_state = {}
		
		# Track previous target angles to detect direction changes
		self.previous_target_angles = {}
		
		# Movement state tracking
		self.movement_state = "idle"  # idle, moving, aligning
		self.last_move_time = 0
		self.per_wheel_previous_power = {name: 0.0 for name in self.wheels.keys()}
		self.power_ramp_rate = 0.2  # Rate of acceleration/deceleration (0-1 per iteration)
		
		# Motor current monitoring for safety and diagnostics
		self.motor_current_max = 40.0  # Amps - threshold for mechanical bind/collision alert
		self.motor_current_sustained_limit = 35.0  # Amps - sustained high current threshold
		self.motor_current_history = {name: [] for name in self.wheels.keys()}
		self.motor_current_max_history = 10  # Track last 10 readings
		self.motor_current_alert_cooldown = 0  # Prevent alert spam
		self.motor_current_alerts = {name: False for name in self.wheels.keys()}
	
	def stop_all(self):
		"""Stop all motors"""
		for wheel in self.wheels.values():
			wheel.stop()
		# Clear any active wheel alignments
		self.wheel_alignment_state.clear()
		# Reset movement state
		self.movement_state = "idle"
		self.per_wheel_previous_power = {name: 0.0 for name in self.wheels.keys()}
	
	def _apply_smooth_acceleration(self, wheel_name, target_power):
		"""Apply smooth acceleration/deceleration to wheel drive power"""
		previous_power = self.per_wheel_previous_power.get(wheel_name, 0.0)
		
		# Smoothly interpolate between previous and target power
		power_diff = target_power - previous_power
		ramped_power = previous_power + (power_diff * self.power_ramp_rate)
		
		# Store for next iteration
		self.per_wheel_previous_power[wheel_name] = ramped_power
		
		return ramped_power
	
	def is_moving(self):
		"""Returns True if robot is currently moving or aligning"""
		return self.movement_state in ["moving", "aligning"]
	
	def is_aligning(self):
		"""Returns True if wheels are currently rotating to target angles"""
		return len(self.wheel_alignment_state) > 0
	
	def get_movement_state(self):
		"""Returns current movement state: 'idle', 'moving', or 'aligning'"""
		return self.movement_state
	
	def update_motor_currents(self):
		"""Update motor current monitoring for all wheels"""
		current_time = wpilib.Timer.getFPGATimestamp()
		
		for wheel_name, wheel in self.wheels.items():
			try:
				# Get current from drive motor
				drive_current = wheel.drive_motor.getOutputCurrent() if wheel.drive_motor else 0.0
				turn_current = wheel.turn_motor.getOutputCurrent() if wheel.turn_motor else 0.0
				
				# Use whichever is higher (usually drive motor draws more)
				motor_current = max(drive_current, turn_current)
				
				# Track history
				self.motor_current_history[wheel_name].append({
					"current": motor_current,
					"time": current_time
				})
				
				# Keep only last N readings
				if len(self.motor_current_history[wheel_name]) > self.motor_current_max_history:
					self.motor_current_history[wheel_name].pop(0)
				
				# Check for high current conditions
				if motor_current > self.motor_current_max:
					# Instantaneous spike - potential collision
					if not self.motor_current_alerts[wheel_name]:
						print(f"[CURRENT-ALERT] {wheel_name}: SPIKE {motor_current:.1f}A (collision/bind?)", flush=True)
						self.motor_current_alerts[wheel_name] = True
						self.motor_current_alert_cooldown = 10  # Cooldown counter
				elif motor_current > self.motor_current_sustained_limit:
					# High sustained current - check average
					avg_current = sum([r["current"] for r in self.motor_current_history[wheel_name]]) / len(self.motor_current_history[wheel_name])
					if avg_current > self.motor_current_sustained_limit:
						if not self.motor_current_alerts[wheel_name]:
							print(f"[CURRENT-ALERT] {wheel_name}: SUSTAINED HIGH {avg_current:.1f}A avg (mechanical issue?)", flush=True)
							self.motor_current_alerts[wheel_name] = True
							self.motor_current_alert_cooldown = 10
				else:
					# Current returned to normal
					if self.motor_current_alerts[wheel_name]:
						print(f"[CURRENT-CLEAR] {wheel_name}: Current normalized", flush=True)
						self.motor_current_alerts[wheel_name] = False
				
				# Publish to SmartDashboard for telemetry
				SmartDashboard.putNumber(f"{wheel_name}_motor_current", motor_current)
				SmartDashboard.putBoolean(f"{wheel_name}_current_alert", self.motor_current_alerts[wheel_name])
				
			except Exception as e:
				pass  # Encoder not responding or other issue
		
		# Decrement cooldown
		if self.motor_current_alert_cooldown > 0:
			self.motor_current_alert_cooldown -= 1
	
	def get_motor_current(self, wheel_name):
		"""Get current motor current (amps) for a wheel"""
		if wheel_name in self.motor_current_history and self.motor_current_history[wheel_name]:
			return self.motor_current_history[wheel_name][-1]["current"]
		return 0.0
	
	def has_current_alert(self, wheel_name=None):
		"""Check if any wheel has current alert, or specific wheel if provided"""
		if wheel_name:
			return self.motor_current_alerts.get(wheel_name, False)
		return any(self.motor_current_alerts.values())
	
	def drive_straight(self, speed, target_angle=0.0):
		"""
		Drive straight at specified speed while maintaining heading angle (autonomous)
		Prevents sideways drift by holding wheel angles fixed.
		
		Args:
			speed: Drive speed (-1.0 to 1.0), positive = forward
			target_angle: Heading to maintain (0-360), 0 = forward
		"""
		if abs(speed) < 0.01:
			# Stop
			for wheel_name, wheel in self.wheels.items():
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
			return
		
		# Point all wheels forward (or target angle)
		import math
		for wheel_name, wheel_pos in config.WHEEL_POSITIONS.items():
			# Set all wheels to same target angle for straight movement
			target_wheel_angle = target_angle
			
			# Apply offset
			if wheel_name in config.MANUAL_OFFSETS:
				target_wheel_angle = (target_wheel_angle + config.MANUAL_OFFSETS[wheel_name]) % 360
			
			# Queue alignment
			self.drive_wheel_to_angle(wheel_name, target_wheel_angle)
			
			# Get current wheel
			wheel = self.wheels[wheel_name]
			current_angle = wheel.get_angle()
			angle_error = abs(target_wheel_angle - current_angle)
			if angle_error > 180:
				angle_error = 360 - angle_error
			
			# Only drive if aligned
			tolerance = config.ALIGN_TOLERANCE
			if angle_error <= tolerance:
				drive_power = -speed * config.MOTOR_SCALE_TELEOP
				ramped_power = self._apply_smooth_acceleration(wheel_name, drive_power)
				wheel.set_drive_power(ramped_power)
				self.movement_state = "moving"
				self.last_move_time = wpilib.Timer.getFPGATimestamp()
			else:
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
				self.movement_state = "aligning"
		
		self.update_single_wheel_alignment()
		self.update_motor_currents()
	
	def drive_to_heading(self, target_angle):
		"""
		Rotate in place to specific heading without forward movement (autonomous)
		
		Args:
			target_angle: Target heading angle (0-360)
		
		Returns:
			True if aligned to within ALIGN_TOLERANCE, False if still rotating
		"""
		# Point all wheels to rotation angles for in-place spin
		base_angles = config.ROTATION_ANGLES
		
		# Determine rotation direction
		angle_diff = target_angle - 0  # Assuming 0 is current forward
		if angle_diff > 180:
			angle_diff -= 360
		elif angle_diff < -180:
			angle_diff += 360
		
		# Use rotation angles (full power)
		if angle_diff >= 0:
			angles = base_angles  # Clockwise
		else:
			angles = {name: (angle + 180) % 360 for name, angle in base_angles.items()}  # Counter-clockwise
		
		rotation_speed = 0.5  # Moderate rotation speed for heading control
		drive_power = -abs(rotation_speed) * config.MOTOR_SCALE_TELEOP
		
		all_aligned = True
		for wheel_name, target_wheel_angle in angles.items():
			# Apply offset
			if wheel_name in config.MANUAL_OFFSETS:
				target_wheel_angle = (target_wheel_angle + config.MANUAL_OFFSETS[wheel_name]) % 360
			
			self.drive_wheel_to_angle(wheel_name, target_wheel_angle)
			
			wheel = self.wheels[wheel_name]
			current_angle = wheel.get_angle()
			angle_error = abs(target_wheel_angle - current_angle)
			if angle_error > 180:
				angle_error = 360 - angle_error
			
			tolerance = config.ALIGN_TOLERANCE
			if angle_error <= tolerance:
				ramped_power = self._apply_smooth_acceleration(wheel_name, drive_power)
				wheel.set_drive_power(ramped_power)
			else:
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
				all_aligned = False
				self.movement_state = "aligning"
		
		self.update_single_wheel_alignment()
		self.update_motor_currents()
		
		return all_aligned
	
	def drive_for_distance(self, speed, target_distance_cm, target_angle=0.0):
		"""
		Drive forward a specific distance at specified speed (autonomous)
		Uses odometry to track distance.
		
		Args:
			speed: Drive speed (-1.0 to 1.0)
			target_distance_cm: Distance to travel in centimeters
			target_angle: Heading to maintain during movement
		
		Returns:
			True if distance reached, False if still moving
		"""
		# Get distance traveled from odometry
		current_distance = self.odometry.get_distance_traveled()
		remaining_distance = target_distance_cm - current_distance
		
		if remaining_distance <= 0:
			# Distance reached - stop
			self.drive_straight(0.0, target_angle)
			return True
		
		# Ramp down as we approach target (smooth deceleration)
		decel_distance = 20.0  # Cm to start decelerating
		if remaining_distance < decel_distance:
			# Linear ramp from speed to 0
			ramped_speed = speed * (remaining_distance / decel_distance)
		else:
			ramped_speed = speed
		
		# Drive at ramped speed
		self.drive_straight(ramped_speed, target_angle)
		return False
	
	def drive_speed_ramped(self, target_speed, ramp_rate=0.1):
		"""
		Continuously apply smooth speed ramping for stable autonomous acceleration
		Call this repeatedly to reach target speed smoothly.
		
		Args:
			target_speed: Target speed to ramp toward (-1.0 to 1.0)
			ramp_rate: Rate of speed change per loop iteration (0.1 = 10% per call)
		
		Returns:
			Current ramped speed
		"""
		if not hasattr(self, '_current_ramp_speed'):
			self._current_ramp_speed = 0.0
		
		# Smoothly interpolate toward target
		speed_diff = target_speed - self._current_ramp_speed
		self._current_ramp_speed += speed_diff * ramp_rate
		
		return self._current_ramp_speed
	
	def drive_swerve(self, forward, strafe, rotate):
		"""
		Drive robot using swerve kinematics
		
		Control logic:
		- Rotation wheels: Always point to rotation target at 100% power
		- Drive wheels: Move in movement direction at speed = MAX(left_stick_magnitude, right_stick_magnitude)
		
		Args:
			forward: Forward speed (-1.0 to 1.0)
			strafe: Strafe speed (-1.0 to 1.0), positive = right
			rotate: Rotation speed (-1.0 to 1.0), positive = counter-clockwise
		"""
		import math
		import time
		
		# Apply deadzone to each input individually to eliminate stick noise
		deadzone = 0.1
		forward = forward if abs(forward) >= deadzone else 0.0
		strafe = strafe if abs(strafe) >= deadzone else 0.0
		rotate = rotate if abs(rotate) >= deadzone else 0.0
		
		# If nothing passed deadzone, stop processing
		if forward == 0 and strafe == 0 and rotate == 0:
			# Ramp down to idle
			for wheel_name, wheel in self.wheels.items():
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
			if not self.is_aligning():
				self.movement_state = "idle"
			return
		
		forward *= -1
		strafe *= -1
		
		# Calculate stick magnitudes
		left_stick_magnitude = math.sqrt(forward*forward + strafe*strafe)
		right_stick_magnitude = abs(rotate)
		drive_multiplier = max(left_stick_magnitude, right_stick_magnitude)
		
		# Debug input values only when non-zero
		if (forward != 0 or strafe != 0 or rotate != 0):
			if not hasattr(self, '_last_input') or self._last_input != (forward, strafe, rotate):
				#print(f"[DRIVE] Input: forward={forward:.3f}, strafe={strafe:.3f}, rotate={rotate:.3f}, multiplier={drive_multiplier:.3f}", flush=True)
				self._last_input = (forward, strafe, rotate)
		
		# Calculate wheel vectors using swerve kinematics (for direction)
		# Rotation contributes to direction but NOT to speed magnitude
		wheel_vectors = {}
		
		for wheel_name, wheel_pos in config.WHEEL_POSITIONS.items():
			# Kinematics: combine forward/strafe with rotation offset for DIRECTION
			vx = -forward - rotate * wheel_pos["y"]
			vy = strafe + rotate * wheel_pos["x"]
			
			# Calculate angle from movement vector
			angle = math.degrees(math.atan2(vy, vx))
			
			# Normalize angle to 0-360
			if angle < 0:
				angle += 360
			
			angle = (angle + 180) % 360  # Invert all inputs globally
			
			# Apply physical mounting offset to kinematics angle
			if wheel_name in config.MANUAL_OFFSETS:
				angle = (angle + config.MANUAL_OFFSETS[wheel_name]) % 360
			
			# Speed calculated from movement inputs only (forward/strafe), scaled by stick dominance
			# movement_speed comes from the magnitude of left stick input
			# drive_multiplier is MAX(left_stick_magnitude, right_stick_magnitude)
			# This way all wheels move at same speed, controlled by whichever stick is pushed harder
			movement_speed = math.sqrt(forward*forward + strafe*strafe)
			wheel_speed = movement_speed * drive_multiplier if drive_multiplier > 0 else 0
			
			wheel_vectors[wheel_name] = {"speed": wheel_speed, "angle": angle}
		
		# Debug: show angles only when they change
		angle_hash = tuple(sorted([(name, round(data['angle'], 1)) for name, data in wheel_vectors.items()]))
		if not hasattr(self, '_last_angle_hash') or self._last_angle_hash != angle_hash:
			#print(f"[DRIVE] Target angles: {dict(angle_hash)}", flush=True)
			self._last_angle_hash = angle_hash
		
		# Coordinate wheel speed limiting: find max speed and scale all wheels proportionally
		max_wheel_speed = max([data['speed'] for data in wheel_vectors.values()]) if wheel_vectors else 0
		if max_wheel_speed > 1.0:
			# Scale all wheels down proportionally to keep them coordinated
			speed_scale = 1.0 / max_wheel_speed
			for wheel_data in wheel_vectors.values():
				wheel_data['speed'] *= speed_scale
		
		# Apply to wheels - drive and turn simultaneously
		for wheel_name, wheel_data in wheel_vectors.items():
			wheel = self.wheels[wheel_name]
			target_angle = wheel_data["angle"]
			target_angle = (target_angle + 180) % 360  # Flip 0° and 180° positions
			target_speed = wheel_data["speed"]
			
			# Get current angle for debugging
			current_angle = wheel.get_angle()
			angle_error = abs(target_angle - current_angle)
			if angle_error > 180:
				angle_error = 360 - angle_error
			
			# Store target angle for alignment ONLY if there's movement and angle changed
			if target_speed > 0.01 and (wheel_name not in self.previous_target_angles or self.previous_target_angles[wheel_name] != target_angle):
				self.wheel_alignment_state[wheel_name] = {
					"target_angle": target_angle,
					"start_time": wpilib.Timer.getFPGATimestamp()
				}
				self.previous_target_angles[wheel_name] = target_angle
			
			# Only apply drive power if wheel is close to target angle (within tolerance)
			# This prevents the wheel from driving in the wrong direction while still rotating
			tolerance = config.ALIGN_TOLERANCE
			if angle_error <= tolerance:
				# Wheel is aligned - apply drive power with smooth ramping
				drive_power = -target_speed * config.MOTOR_SCALE_TELEOP
				ramped_power = self._apply_smooth_acceleration(wheel_name, drive_power)
				wheel.set_drive_power(ramped_power)
				if target_speed > 0.01:
					self.movement_state = "moving"
					self.last_move_time = wpilib.Timer.getFPGATimestamp()
			else:
				# Wheel is still rotating to target - don't drive yet
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
				if len(self.wheel_alignment_state) > 0:
					self.movement_state = "aligning"
		
		# Monitor motor currents for safety
		self.update_motor_currents()
	
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
	
	def get_wheel_power(self, wheel_name):
		"""Get drive power of specific wheel"""
		if wheel_name in self.wheels:
			return self.wheels[wheel_name].get_drive_power()
		return 0.0
	
	def set_wheel_zero(self, wheel_name):
		"""Save current position as zero for a wheel"""
		if wheel_name in self.wheels:
			wheel = self.wheels[wheel_name]
			wheel.set_zero_offset(wheel.get_raw_angle())
			self.calibration.set_offset(wheel_name, wheel.offset)
			self.calibration.save_offsets()
	
	def set_wheel_angle(self, wheel_name, target_angle):
		"""Save current position as a specific angle (0, 90, 180, 270)"""
		if wheel_name in self.wheels:
			wheel = self.wheels[wheel_name]
			raw = wheel.get_raw_angle()
			# Calculate offset: offset = raw - target
			calculated_offset = (raw - target_angle) % 360
			
			wheel.offset = calculated_offset
			self.calibration.set_offset(wheel_name, calculated_offset)
			self.calibration.save_offsets()
			
			for name, offset in self.calibration.offsets.items():
				self.wheels[name].offset = offset
			
			verify_angle = (raw - wheel.offset) % 360
	
	def drive_wheel_to_angle(self, wheel_name, target_angle):
		"""Start aligning a single wheel to target angle using PID control"""
		if wheel_name not in self.wheels:
			#print(f"[DRIVE-WHEEL] Unknown wheel: {wheel_name}")
			return
		
		# Set up continuous alignment for this wheel (only if not already aligning to same angle)
		if wheel_name not in self.wheel_alignment_state or self.wheel_alignment_state[wheel_name]["target_angle"] != target_angle:
			self.wheel_alignment_state[wheel_name] = {
				"target_angle": target_angle,
				"start_time": wpilib.Timer.getFPGATimestamp()
			}
	
	
	def start_alignment(self, target_angle=0):
		"""Start aligning all wheels to target angle"""
		# Clear per-wheel alignment state so only global alignment runs
		self.wheel_alignment_state.clear()
		
		self.aligning = True
		self.align_start_time = wpilib.Timer.getFPGATimestamp()
		self.target_align_angle = target_angle
	
	def update_alignment(self):
		"""Update alignment routine (call every loop)"""
		if not self.aligning:
			return
		
		elapsed = wpilib.Timer.getFPGATimestamp() - self.align_start_time
		
		if elapsed < config.ALIGN_TIMEOUT:
			all_aligned = True
			current_time = wpilib.Timer.getFPGATimestamp()
			
			for wheel_name, wheel in self.wheels.items():
				current_angle = wheel.get_angle()
				
				# Normalize both angles to -180 to 180 range first
				# This prevents boundary issues at 360/0
				norm_current = current_angle
				if norm_current > 180:
					norm_current -= 360
				
				norm_target = self.target_align_angle
				if norm_target > 180:
					norm_target -= 360
				
				# Now calculate shortest path to target
				error = norm_target - norm_current
				# Normalize to -180 to 180
				if error > 180:
					error -= 360
				elif error < -180:
					error += 360
				
				# Use PID controller for smooth control
				pid = self.pid_controllers[wheel_name]
				pid_output = pid.calculate(error, current_time)
				
				# Clamp output to motor scale
				speed = max(-config.MOTOR_SCALE_ALIGN, 
						   min(config.MOTOR_SCALE_ALIGN, pid_output))
				

				
				# Always apply PID output (don't cut to 0 when within tolerance)
				# PID naturally outputs near-zero as error approaches zero
				wheel.set_turn_power(speed)
				
				if abs(error) > config.ALIGN_TOLERANCE:
					all_aligned = False
			
			self.align_debug_counter += 1
			
			if all_aligned:
				self.aligning = False
				# Reset PID controllers for next alignment
				for pid in self.pid_controllers.values():
					pid.reset()
				self.align_debug_counter = 0
		else:
			self.stop_all()
			self.aligning = False
			# Reset PID controllers
			for pid in self.pid_controllers.values():
				pid.reset()
			self.align_debug_counter = 0
	
	def update_single_wheel_alignment(self):
		"""Update per-wheel alignment (call every loop)"""
		if not self.wheel_alignment_state:
			return
		
		# Process all wheels in alignment state
		wheels_to_remove = []
		
		for wheel_name, align_info in self.wheel_alignment_state.items():
			target_angle = align_info["target_angle"]
			elapsed = wpilib.Timer.getFPGATimestamp() - align_info["start_time"]
			
			if wheel_name not in self.wheels:
				wheels_to_remove.append(wheel_name)
				continue
			
			wheel = self.wheels[wheel_name]
			raw_angle = wheel.get_raw_angle()
			current_angle = wheel.get_angle()
			
			# Calculate error between target and current angle
			raw_error = target_angle - current_angle
			
			# Simple shortest-path wrapping: 
			# If error is > 180, go the other direction instead
			# If error is < -180, go the other direction instead
			if raw_error > 180:
				error = raw_error - 360
			elif raw_error < -180:
				error = raw_error + 360
			else:
				error = raw_error
			
			# Get PID controller for this wheel
			pid = self.pid_controllers[wheel_name]
			
			# Check if we're close enough (within tolerance - prevents oscillation)
			tolerance = config.ALIGN_TOLERANCE
			at_target = abs(error) < tolerance
			
			if at_target:
				# Aligned! Stop only the TURN motor and RESET ITS PID (kill residual power/integral)
				#if wheel_name == "front_right":
				#	print(f"[FR] *** REACHED TOLERANCE *** Stopping turn motor. Final angle: {current_angle:.1f}°", flush=True)
				wheel.turn_motor.set(0.0)
				pid.reset()  # Reset PID to clear integral and derivative terms
				wheels_to_remove.append(wheel_name)
				continue
			
			# Check timeout (30 seconds per wheel)
			if elapsed > 30:
				wheel.stop()
				wheels_to_remove.append(wheel_name)
				continue
			
			# Use PID controller to maintain or reach target angle
			pid = self.pid_controllers[wheel_name]
			current_time = wpilib.Timer.getFPGATimestamp()
			pid_output = pid.calculate(error, current_time)
			
			# Clamp output to motor scale range
			speed = max(-config.MOTOR_SCALE_ALIGN, min(config.MOTOR_SCALE_ALIGN, pid_output))
			
			wheel.set_turn_power(speed)
		
		# Clean up finished wheels
		for wheel_name in wheels_to_remove:
			del self.wheel_alignment_state[wheel_name]
	
	def drive_rotation(self, rotation_input):
		"""
		Drive robot in place with 360° rotation using verified tangent wheel angles.
		Wheels point outward at 45° angles to create circular motion.
		
		Args:
			rotation_input: Right stick X value (-1.0 to 1.0)
						   Negative = counter-clockwise (left), Positive = clockwise (right)
						   Rotation wheels align at 100%, drive wheels move at stick magnitude
		"""
		# Deadzone
		if abs(rotation_input) < 0.1:
			# Ramp down to idle
			for wheel_name, wheel in self.wheels.items():
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
			if not self.is_aligning():
				self.movement_state = "idle"
			return
		
		# Base rotation angles (clockwise) - wheels point outward to create clockwise spin
		base_angles = config.ROTATION_ANGLES
		
		# Select angle set based on input direction
		# Counter-clockwise (left): add 180° to each wheel angle
		if rotation_input <= 0:
			angles = {name: (angle + 180) % 360 for name, angle in base_angles.items()}
		else:
			angles = base_angles
		
		# Apply rotation angles to all wheels and spin at speed scaled by stick magnitude
		# Rotation wheels snap to angles at 100%, drive wheels move at rotation_input magnitude
		drive_power = -abs(rotation_input) * config.MOTOR_SCALE_TELEOP
		
		for wheel_name, target_angle in angles.items():
			# Apply physical mounting offset to rotation angle
			if wheel_name in config.MANUAL_OFFSETS:
				target_angle = (target_angle + config.MANUAL_OFFSETS[wheel_name]) % 360
			
			# Point wheel to target angle
			self.drive_wheel_to_angle(wheel_name, target_angle)
			
			# Get current angle for alignment check
			wheel = self.wheels[wheel_name]
			current_angle = wheel.get_angle()
			angle_error = abs(target_angle - current_angle)
			if angle_error > 180:
				angle_error = 360 - angle_error
			
			# Only apply drive power if wheel is close to target angle (within tolerance)
			# This prevents the wheel from driving in the wrong direction while still rotating
			tolerance = config.ALIGN_TOLERANCE
			if angle_error <= tolerance:
				# Wheel is aligned - apply drive power
				ramped_power = self._apply_smooth_acceleration(wheel_name, drive_power)
				wheel.set_drive_power(ramped_power)
				if abs(rotation_input) > 0.01:
					self.movement_state = "moving"
					self.last_move_time = wpilib.Timer.getFPGATimestamp()
			else:
				# Wheel is still rotating to target - don't drive yet
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
				if len(self.wheel_alignment_state) > 0:
					self.movement_state = "aligning"
		
		# Monitor motor currents for safety
		self.update_motor_currents()
		
		# Keep wheels aligned during rotation
		self.update_single_wheel_alignment()

	def _debug_print_turn_motor_powers(self):
		"""Print turn motor powers for all wheels (call every loop to monitor)"""
		# Only print front_right to avoid spam
		if "front_right" in self.wheels:
			wheel = self.wheels["front_right"]
			try:
				turn_power = wheel.turn_motor.get() if wheel.turn_motor else 0.0
			except:
				turn_power = 0.0
			
			# Only print if it's not zero (or every 20 loops if zero)
			if not hasattr(self, '_fr_turn_power_debug'):
				self._fr_turn_power_debug = 0
			
			self._fr_turn_power_debug += 1
			if turn_power != 0.0 or self._fr_turn_power_debug >= 20:
				#if turn_power != 0.0:
				#	print(f"[FR-TURN-CHECK] Motor power: {turn_power:.4f} (SHOULD BE 0!)", flush=True)
				self._fr_turn_power_debug = 0
	
	def _debug_print_drive_motor_diagnostics(self):
		"""Diagnostic for FR drive motor at full power"""
		if "front_right" not in self.wheels:
			return
		
		wheel = self.wheels["front_right"]
		if not wheel.drive_motor:
			return
		
		try:
			# Get all motor diagnostics
			drive_power_set = wheel.current_drive_power  # What we commanded
			drive_power_actual = wheel.drive_motor.get()  # What the motor reports
			
			encoder = wheel.drive_motor.getEncoder()
			drive_velocity = encoder.getVelocity() if encoder else 0
			drive_position = encoder.getPosition() if encoder else 0
			
			# Check motor current (indicates mechanical bind if very high)
			try:
				motor_current = wheel.drive_motor.getOutputCurrent()
			except:
				motor_current = 0.0
			
			# Check motor temperature
			try:
				motor_temp = wheel.drive_motor.getMotorTemperature()
			except:
				motor_temp = 0.0
			
			turn_power = wheel.turn_motor.get() if wheel.turn_motor else 0.0
			
			# Print only once per ~20 loops at full power
			if not hasattr(self, '_fr_drive_diag_counter'):
				self._fr_drive_diag_counter = 0
			
			self._fr_drive_diag_counter += 1
			if self._fr_drive_diag_counter >= 20 and wheel.current_drive_power > 0.9:
				self._fr_drive_diag_counter = 0
		except Exception as e:
			pass  # Ignore errors
	
	def start_autotune(self):
		"""Start oscillation-based autotune at 4 angles for all 4 wheels"""
		self.autotuning = True
		self.autotune_gains = {
			"wheels": ["front_left", "front_right", "rear_left", "rear_right"],
			"current_index": 0,
			"results": [],
			"wheel_start_time": wpilib.Timer.getFPGATimestamp(),
			"current": {
				"angles": [0, 90, 180, 270],  # Test 4 angles per wheel
				"angle_index": 0,
				"kc_list": [],  # Critical gains for each angle
				"tc_list": [],  # Critical periods for each angle
				"sign_changes": 0,
				"last_error": 0,
				"kp": 0.009,  # Start lower so we can see KP escalation
				"angle_start_time": wpilib.Timer.getFPGATimestamp(),
				"step": 0
			}
		}
	
	def update_autotune(self):
		"""Update autotune - test oscillations at 4 angles to find Kc and Tc"""
		if not self.autotuning:
			return
		
		gains = self.autotune_gains
		current_idx = gains["current_index"]
		wheel_name = gains["wheels"][current_idx]
		wheel = self.wheels[wheel_name]
		
		wheel_elapsed = wpilib.Timer.getFPGATimestamp() - gains["wheel_start_time"]
		
		# Timeout: 30 seconds per wheel (6 sec x 4 angles + buffer)
		if wheel_elapsed > 30:
			self._finalize_wheel_autotune()
			return
		
		current = gains["current"]
		angle_index = current["angle_index"]
		target_angle = current["angles"][angle_index]
		
		# Timeout for each angle: 6 seconds
		angle_elapsed = wpilib.Timer.getFPGATimestamp() - current["angle_start_time"]
		if angle_elapsed > 6:
			# Move to next angle without finding Kc (use None as marker)
			current["kc_list"].append(None)
			current["tc_list"].append(None)
			self._next_angle_or_finalize()
			return
		
		# Get current angle and calculate error
		current_angle = wheel.get_angle()
		error = target_angle - current_angle
		if error > 180:
			error -= 360
		elif error < -180:
			error += 360
		
		# Track sign changes for oscillation detection
		if error * current["last_error"] < 0:  # Sign changed
			current["sign_changes"] += 1
		current["last_error"] = error
		
		# P control at current KP
		speed = max(-0.5, min(0.5, error * current["kp"]))
		wheel.set_turn_power(speed)
		
		# Increase KP every 1.2 seconds to find oscillation
		step = int(angle_elapsed / 1.2)
		if step > current["step"]:
			current["step"] = step
			old_kp = current["kp"]
			current["kp"] *= 1.5
		
		# When we detect oscillation (8+ sign changes), record Kc
		if current["sign_changes"] >= 8 and len(current["kc_list"]) == angle_index:
			kc = current["kp"]
			current["kc_list"].append(kc)
			current["osc_start"] = wpilib.Timer.getFPGATimestamp()
			current["sign_changes"] = 0  # Reset for period measurement
		
		# Measure period: wait for 8 MORE sign changes after finding Kc
		if len(current["kc_list"]) == angle_index + 1 and len(current["tc_list"]) == angle_index:
			if current["sign_changes"] >= 8:  # 4 full cycles for period
				period_time = wpilib.Timer.getFPGATimestamp() - current["osc_start"]
				tc = period_time / 4.0
				current["tc_list"].append(tc)
				# Move to next angle
				self._next_angle_or_finalize()
	
	def _next_angle_or_finalize(self):
		"""Move to next angle or finalize wheel"""
		gains = self.autotune_gains
		current = gains["current"]
		current_idx = gains["current_index"]
		wheel_name = gains["wheels"][current_idx]
		
		current["angle_index"] += 1
		
		if current["angle_index"] < len(current["angles"]):
			# More angles to test
			current["angle_start_time"] = wpilib.Timer.getFPGATimestamp()
			current["kp"] = 0.009  # Reset KP for next angle
			current["sign_changes"] = 0
			current["last_error"] = 0
			current["step"] = 0
		else:
			# All angles done for this wheel
			self._finalize_wheel_autotune()

	
	def _finalize_wheel_autotune(self):
		"""Finalize one wheel's autotune by averaging results from 4 angles"""
		gains = self.autotune_gains
		current = gains["current"]
		current_idx = gains["current_index"]
		wheel_name = gains["wheels"][current_idx]
		self.wheels[wheel_name].set_turn_power(0)
		
		# Average Kc and Tc across all valid tests
		valid_kc = [kc for kc in current["kc_list"] if kc is not None]
		valid_tc = [tc for tc in current["tc_list"] if tc is not None]
		
		if valid_kc and valid_tc:
			avg_kc = sum(valid_kc) / len(valid_kc)
			avg_tc = sum(valid_tc) / len(valid_tc)
			
			# Use Ziegler-Nichols formulas (reduced KI multiplier from 1.2 to 0.8)
			kp = 0.6 * avg_kc
			ki = 0.8 * avg_kc / avg_tc if avg_tc > 0 else 0
			kd = 0.075 * avg_kc * avg_tc
		else:
			# Failed to find oscillation at any angle
			kp, ki, kd = 0.01, 0.002, 0.0001
		
		gains["results"].append({"wheel": wheel_name, "kp": kp, "ki": ki, "kd": kd})
		
		# Move to next wheel or finalize
		current_idx += 1
		if current_idx < len(gains["wheels"]):
			gains["current_index"] = current_idx
			gains["wheel_start_time"] = wpilib.Timer.getFPGATimestamp()
			gains["current"] = {
				"angles": [0, 90, 180, 270],
				"angle_index": 0,
				"kc_list": [],
				"tc_list": [],
				"sign_changes": 0,
				"last_error": 0,
				"kp": 0.009,
				"angle_start_time": wpilib.Timer.getFPGATimestamp(),
				"step": 0
			}
		else:
			self._compute_final_gains()
	
	def _compute_final_gains(self):
		"""Store per-wheel results and apply to all wheels"""
		gains = self.autotune_gains
		results = gains["results"]
		
		# Store per-wheel gains (don't average)
		wheel_gains = {}
		for r in results:
			wheel_gains[r['wheel']] = {
				"kp": r['kp'],
				"ki": r['ki'],
				"kd": r['kd']
			}
		
		# Apply per-wheel gains to controllers
		for wheel_name, pid in self.pid_controllers.items():
			if wheel_name in wheel_gains:
				g = wheel_gains[wheel_name]
				pid.set_gains(g['kp'], g['ki'], g['kd'])
		
		# Save with battery voltage for interpolation
		battery_voltage = RobotController.getBatteryVoltage()
		
		# Add as tuning result with per-wheel data
		self.calibration.add_tuning_result(battery_voltage, wheel_gains)
		self.calibration.save_calibration()
		
		# Update NetworkTables for dashboard feedback (publish per-wheel gains)
		for wheel_name, gains in wheel_gains.items():
			SmartDashboard.putNumber(f"autotune_{wheel_name}_kp", gains['kp'])
			SmartDashboard.putNumber(f"autotune_{wheel_name}_ki", gains['ki'])
			SmartDashboard.putNumber(f"autotune_{wheel_name}_kd", gains['kd'])
		SmartDashboard.putNumber("autotune_battery_voltage", battery_voltage)
		SmartDashboard.putBoolean("autotune_complete", True)
		
		# Publish tuning history to NetworkTables for remote dashboard access
		self._publish_tuning_history_to_nt()
		
		self.autotuning = False
		self.autotune_gains = None
	
	def _publish_tuning_history_to_nt(self):
		"""Publish tuning history and regression to NetworkTables for remote dashboard access"""
		import sys
		import json
		
		try:
			# Get tuning history
			history = self.calibration.pid_tuning_history
			
			# Serialize and publish
			history_json = json.dumps(history)
			
			SmartDashboard.putString("autotune_history_json", history_json)
			
			# Also publish regression if available
			if hasattr(self.calibration, 'pid_regression') and self.calibration.pid_regression:
				regression_json = json.dumps(self.calibration.pid_regression)
				SmartDashboard.putString("autotune_regression_json", regression_json)
				
		except Exception as e:
			import traceback
			traceback.print_exc()
