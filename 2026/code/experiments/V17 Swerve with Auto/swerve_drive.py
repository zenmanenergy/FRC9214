"""Swerve drive controller for all 4 wheels"""
import wpilib
from wpilib import SmartDashboard, RobotController
from wheel import SwerveWheel
from encoder_calibration import EncoderCalibration
from pid_controller import PIDController
from swerve_odometry import SwerveOdometry
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
			# Only apply offsets to actual wheels (skip turret offsets)
			if wheel_name in self.wheels:
				self.wheels[wheel_name].offset = offset
		
		print("[ROBOT] Initialized 4 wheels with offsets")
		
		# Alignment state
		self.aligning = False
		self.align_start_time = None
		self.target_align_angle = 0  # Target angle for alignment
		self.align_debug_counter = 0  # Counter for debug output
		
		# Load PID gains from calibration file with voltage interpolation
		battery_voltage = RobotController.getBatteryVoltage()
		pid_gains = self.calibration.get_interpolated_gains(battery_voltage)
		
		import sys
		print(f"\n{'='*60}")
		print(f"[ROBOT] LOADING PID GAINS")
		print(f"[ROBOT] Battery voltage: {battery_voltage:.2f}V")
		sys.stdout.flush()
		
		# Create PID controllers for each wheel with per-wheel gains if available
		self.pid_controllers = {}
		for wheel_name in self.wheels.keys():
			# Check if we have per-wheel gains
			if isinstance(pid_gains, dict) and wheel_name in pid_gains:
				kp = pid_gains[wheel_name]["kp"]
				ki = pid_gains[wheel_name]["ki"]
				kd = pid_gains[wheel_name]["kd"]
				print(f"[ROBOT] {wheel_name:12}: KP={kp:.6f}, KI={ki:.6f}, KD={kd:.6f}")
			else:
				# Fallback to uniform gains
				kp = pid_gains.get("kp", 0.003)
				ki = pid_gains.get("ki", 0.005)
				kd = pid_gains.get("kd", 0.0001)
				print(f"[ROBOT] {wheel_name:12}: KP={kp:.6f}, KI={ki:.6f}, KD={kd:.6f} (uniform)")
			
			self.pid_controllers[wheel_name] = PIDController(
				kp=kp,
				ki=ki,
				kd=kd,
				name=f"Wheel_{wheel_name}"
			)
		print(f"{'-'*60}\n")
		sys.stdout.flush()
		
		# Initialize odometry tracking for distance calculation
		self.odometry = SwerveOdometry(self.wheels, wheel_diameter_cm=10.16)
		print("[ROBOT] Odometry initialized (10.16 cm wheels, 42 CPR NEO 1.1)")
		sys.stdout.flush()
		
		# Publish tuning history to NetworkTables on startup (for remote dashboard)
		print("[ROBOT] About to call _publish_tuning_history_to_nt()...")
		import sys
		sys.stdout.flush()
		self._publish_tuning_history_to_nt()
		sys.stdout.flush()
		
		# Autotune state
		self.autotuning = False
		self.autotune_wheel = None
		self.autotune_gains = None
		self.autotune_start_time = None
		
		# Per-wheel alignment state (dict so multiple wheels can align simultaneously)
		self.wheel_alignment_state = {}
		
		# Track previous target angles to detect direction changes
		self.previous_target_angles = {}
	
	def stop_all(self):
		"""Stop all motors"""
		for wheel in self.wheels.values():
			wheel.stop()
		# Clear any active wheel alignments
		self.wheel_alignment_state.clear()
	
	def drive_swerve(self, forward, strafe, rotate):
		"""
		Drive robot using swerve kinematics
		
		Args:
			forward: Forward speed (-1.0 to 1.0)
			strafe: Strafe speed (-1.0 to 1.0), positive = right
			rotate: Rotation speed (-1.0 to 1.0), positive = counter-clockwise
		"""
		import math
		import time
		
		# Debug input values only when non-zero
		if (forward != 0 or strafe != 0 or rotate != 0):
			if not hasattr(self, '_last_input') or self._last_input != (forward, strafe, rotate):
				print(f"[DRIVE] Input: forward={forward:.3f}, strafe={strafe:.3f}, rotate={rotate:.3f}", flush=True)
				self._last_input = (forward, strafe, rotate)
		
		# Calculate wheel vectors using swerve kinematics
		wheel_vectors = {}
		max_speed = 0
		
		for wheel_name, wheel_pos in config.WHEEL_POSITIONS.items():
			# Swerve kinematics: combine forward/strafe with rotation offset
			# Rotation contributes perpendicular to the position vector
			vx = forward - rotate * wheel_pos["y"]
			vy = strafe + rotate * wheel_pos["x"]
			
			# Calculate speed and angle
			speed = math.sqrt(vx*vx + vy*vy)
			angle = math.degrees(math.atan2(vy, vx))
			
	
			
			# Normalize angle to 0-360
			if angle < 0:
				angle += 360
			
			wheel_vectors[wheel_name] = {"speed": speed, "angle": angle}
			max_speed = max(max_speed, speed)
		

		
		# Normalize speeds if any exceed 1.0
		if max_speed > 1.0:
			for wheel_data in wheel_vectors.values():
				wheel_data["speed"] /= max_speed
		
		# Debug: show angles only when they change
		angle_hash = tuple(sorted([(name, round(data['angle'], 1)) for name, data in wheel_vectors.items()]))
		if not hasattr(self, '_last_angle_hash') or self._last_angle_hash != angle_hash:
			print(f"[DRIVE] Target angles: {dict(angle_hash)}", flush=True)
			self._last_angle_hash = angle_hash
		
		# Apply to wheels - drive and turn simultaneously
		for wheel_name, wheel_data in wheel_vectors.items():
			wheel = self.wheels[wheel_name]
			target_angle = wheel_data["angle"]
			target_speed = wheel_data["speed"]
			
			# Get current angle for debugging
			current_angle = wheel.get_angle()
			angle_error = abs(target_angle - current_angle)
			if angle_error > 180:
				angle_error = 360 - angle_error
			
			# Apply drive power - don't wait for turn motor to align
			drive_power = target_speed * config.MOTOR_SCALE_TELEOP
			wheel.set_drive_power(drive_power)
			
			# Debug front_right every cycle to see if power is changing
			if wheel_name == "front_right":
				if not hasattr(self, '_fr_drive_debug'):
					self._fr_drive_debug = None
				if self._fr_drive_debug != drive_power:
					print(f"[FR-DRIVE] Power changed to: {drive_power:.3f}", flush=True)
					self._fr_drive_debug = drive_power
			
			# Debug front_right wheel (every 10 loops to avoid spam)
			if wheel_name == "front_right":
				if not hasattr(self, '_fr_debug_counter'):
					self._fr_debug_counter = 0
				self._fr_debug_counter += 1
				
				if self._fr_debug_counter >= 10:
					self._fr_debug_counter = 0
					drive_encoder = wheel.drive_motor.getEncoder()
					drive_pos = drive_encoder.getPosition() if wheel.drive_motor else 0
					drive_vel = drive_encoder.getVelocity() if wheel.drive_motor else 0
					timestamp = wpilib.Timer.getFPGATimestamp()
					print(f"[{timestamp:.2f}s] [FR] Heading: {current_angle:.0f}° | Power: {drive_power:.2f} | Pos: {drive_pos:.1f} | Vel: {drive_vel:.1f}", flush=True)
			
			# Store target angle for alignment ONLY if it changed (don't re-add every cycle)
			if wheel_name not in self.previous_target_angles or self.previous_target_angles[wheel_name] != target_angle:
				self.wheel_alignment_state[wheel_name] = {
					"target_angle": target_angle,
					"start_time": wpilib.Timer.getFPGATimestamp()
				}
				self.previous_target_angles[wheel_name] = target_angle
	
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
			print(f"[ZEROING] Saved offset for {wheel_name}: {wheel.offset:.1f} (0 degrees)")
	
	def set_wheel_angle(self, wheel_name, target_angle):
		"""Save current position as a specific angle (0, 90, 180, 270)"""
		if wheel_name in self.wheels:
			wheel = self.wheels[wheel_name]
			raw = wheel.get_raw_angle()
			# Calculate offset: offset = raw - target
			calculated_offset = (raw - target_angle) % 360
			
			print(f"\n[SETANGLE] === Calibrating {wheel_name} to {target_angle} deg ===")
			print(f"[SETANGLE] Step 1: Read encoder")
			print(f"           raw = {raw:.1f} deg")
			print(f"[SETANGLE] Step 2: Calculate offset")
			print(f"           offset = ({raw:.1f} - {target_angle}) % 360 = {calculated_offset:.1f} deg")
			
			wheel.offset = calculated_offset
			self.calibration.set_offset(wheel_name, calculated_offset)
			print(f"[SETANGLE] Step 3: Save to file")
			self.calibration.save_offsets()
			
			print(f"[SETANGLE] Step 4: Reload all offsets from file")
			for name, offset in self.calibration.offsets.items():
				self.wheels[name].offset = offset
				print(f"           {name}: offset={offset:.1f} deg")
			
			print(f"[SETANGLE] Step 5: Verify calculation")
			verify_angle = (raw - wheel.offset) % 360
			print(f"           ({raw:.1f} - {wheel.offset:.1f}) % 360 = {verify_angle:.1f} deg")
			print(f"[SETANGLE] === Complete ===\n")
	
	def drive_wheel_to_angle(self, wheel_name, target_angle):
		"""Start aligning a single wheel to target angle using PID control"""
		if wheel_name not in self.wheels:
			print(f"[DRIVE-WHEEL] Unknown wheel: {wheel_name}")
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
		print(f"[ALIGN] Starting alignment to {target_angle} deg for all wheels...")
		for wheel_name in self.wheels.keys():
			print(f"  {wheel_name}: moving to {target_angle} deg")
	
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
				print(f"[ALIGN] All wheels aligned to {self.target_align_angle} deg!")
				self.aligning = False
				# Reset PID controllers for next alignment
				for pid in self.pid_controllers.values():
					pid.reset()
				self.align_debug_counter = 0
		else:
			print("[ALIGN] Alignment timeout!")
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
				print(f"[ALIGN] ERROR: Unknown wheel: {wheel_name}, stopping")
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
			
			# Check if we're close enough (within 2 degrees)
			tolerance = 2.0
			at_target = abs(error) < tolerance
			
			if at_target:
				# Aligned! Stop only the TURN motor and RESET ITS PID (kill residual power/integral)
				if wheel_name == "front_right":
					print(f"[FR] *** REACHED TOLERANCE *** Stopping turn motor. Final angle: {current_angle:.1f}°", flush=True)
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
			
			# Debug front_right turn motor
			if wheel_name == "front_right":
				print(f"[FR-TURN] Err={error:+.2f}° | PID={pid_output:+.5f} | Speed={speed:+.4f}", flush=True)
		
		# Clean up finished wheels
		for wheel_name in wheels_to_remove:
			del self.wheel_alignment_state[wheel_name]
	
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
				if turn_power != 0.0:
					print(f"[FR-TURN-CHECK] Motor power: {turn_power:.4f} (SHOULD BE 0!)", flush=True)
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
				print(f"[FR-DIAG] Power(set={drive_power_set:.3f}, actual={drive_power_actual:.3f}) | Vel={drive_velocity:.2f} RPM | Current={motor_current:.1f}A | Temp={motor_temp:.1f}C | Turn_Motor={turn_power:.4f}", flush=True)
		except Exception as e:
			pass  # Ignore errors
	
	def start_autotune(self):
		"""Start oscillation-based autotune at 4 angles for all 4 wheels"""
		self.autotuning = True
		print(f"[AUTOTUNE] Starting multi-angle oscillation autotune on all 4 wheels...", flush=True)
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
			print(f"[AUTOTUNE] {wheel_name}: Wheel timeout!")
			self._finalize_wheel_autotune()
			return
		
		current = gains["current"]
		angle_index = current["angle_index"]
		target_angle = current["angles"][angle_index]
		
		# Timeout for each angle: 6 seconds
		angle_elapsed = wpilib.Timer.getFPGATimestamp() - current["angle_start_time"]
		if angle_elapsed > 6:
			print(f"[AUTOTUNE] {wheel_name}: Angle {target_angle} deg timeout!")
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
			print(f"[AUTOTUNE] {wheel_name} @ {target_angle} deg: KP {old_kp:.6f} -> {current['kp']:.6f}, oscillations={current['sign_changes']}")
		
		# When we detect oscillation (8+ sign changes), record Kc
		if current["sign_changes"] >= 8 and len(current["kc_list"]) == angle_index:
			kc = current["kp"]
			current["kc_list"].append(kc)
			current["osc_start"] = wpilib.Timer.getFPGATimestamp()
			print(f"[AUTOTUNE] {wheel_name} @ {target_angle} deg: Found Kc={kc:.6f}, measuring period...")
			current["sign_changes"] = 0  # Reset for period measurement
		
		# Measure period: wait for 8 MORE sign changes after finding Kc
		if len(current["kc_list"]) == angle_index + 1 and len(current["tc_list"]) == angle_index:
			if current["sign_changes"] >= 8:  # 4 full cycles for period
				period_time = wpilib.Timer.getFPGATimestamp() - current["osc_start"]
				tc = period_time / 4.0
				current["tc_list"].append(tc)
				print(f"[AUTOTUNE] {wheel_name} @ {target_angle} deg: Tc={tc:.3f}s")
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
			print(f"[AUTOTUNE] {wheel_name}: Moving to {current['angles'][current['angle_index']]} deg...\n")
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
			
			print(f"[AUTOTUNE] {wheel_name}: {len(valid_kc)} angles succeeded")
			print(f"           avg Kc={avg_kc:.6f}, avg Tc={avg_tc:.3f}s")
			print(f"           -> KP={kp:.6f}, KI={ki:.6f}, KD={kd:.6f}")
		else:
			# Failed to find oscillation at any angle
			kp, ki, kd = 0.01, 0.002, 0.0001
			print(f"[AUTOTUNE] {wheel_name}: No oscillation detected. Using safe defaults.")
		
		gains["results"].append({"wheel": wheel_name, "kp": kp, "ki": ki, "kd": kd})
		
		# Move to next wheel or finalize
		current_idx += 1
		if current_idx < len(gains["wheels"]):
			print(f"\n[AUTOTUNE] === Moving to {gains['wheels'][current_idx]} ===\n")
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
		
		print(f"\n[AUTOTUNE] === ALL WHEELS TUNED ===")
		for r in results:
			print(f"[AUTOTUNE] {r['wheel']:12}: KP={r['kp']:.6f}, KI={r['ki']:.6f}, KD={r['kd']:.6f}")
		
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
		print(f"\n[AUTOTUNE] Saving to calibration file...")
		battery_voltage = RobotController.getBatteryVoltage()
		print(f"[AUTOTUNE] Battery voltage at tuning: {battery_voltage:.2f}V")
		
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
		print(f"[AUTOTUNE] Complete! Settings applied to all wheels and saved to file.\n")
	
	def _publish_tuning_history_to_nt(self):
		"""Publish tuning history and regression to NetworkTables for remote dashboard access"""
		import sys
		import json
		
		print("[NT-PUBLISH] STARTING _publish_tuning_history_to_nt")
		sys.stdout.flush()
		
		try:
			# Get tuning history
			history = self.calibration.pid_tuning_history
			print(f"[NT-PUBLISH] Got history: {len(history)} entries")
			sys.stdout.flush()
			
			# Serialize and publish
			history_json = json.dumps(history)
			print(f"[NT-PUBLISH] JSON serialized: {history_json[:100]}...")
			sys.stdout.flush()
			
			SmartDashboard.putString("autotune_history_json", history_json)
			print(f"[NT-PUBLISH] Published {len(history)} entries to NetworkTables")
			sys.stdout.flush()
			
			# Also publish regression if available
			if hasattr(self.calibration, 'pid_regression') and self.calibration.pid_regression:
				regression_json = json.dumps(self.calibration.pid_regression)
				SmartDashboard.putString("autotune_regression_json", regression_json)
				print(f"[NT-PUBLISH] Published regression")
				sys.stdout.flush()
			else:
				print(f"[NT-PUBLISH] No regression data available")
				sys.stdout.flush()
				
		except Exception as e:
			import traceback
			print(f"[NT-PUBLISH] ERROR: {e}")
			traceback.print_exc()
			sys.stdout.flush()


