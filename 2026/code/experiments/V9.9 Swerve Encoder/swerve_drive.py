"""Swerve drive controller for all 4 wheels"""
import wpilib
from wpilib import SmartDashboard, RobotController
from wheel import SwerveWheel
from encoder_calibration import EncoderCalibration
from pid_controller import PIDController
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
		
		# Single-wheel alignment state
		self.aligning_single_wheel = False
		self.single_wheel_name = None
		self.single_wheel_target_angle = None
		self.single_wheel_start_time = None
	
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
	
	def drive_wheel_to_angle(self, wheel_name, target_angle):
		"""Start aligning a single wheel to target angle using PID control"""
		if wheel_name not in self.wheels:
			print(f"[DRIVE-WHEEL] Unknown wheel: {wheel_name}")
			return
		
		# Set up continuous alignment for this wheel
		self.aligning_single_wheel = True
		self.single_wheel_name = wheel_name
		self.single_wheel_target_angle = target_angle
		self.single_wheel_start_time = wpilib.Timer.getFPGATimestamp()
		print(f"[DRIVE-WHEEL] Starting alignment of {wheel_name} to {target_angle}°")
	
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
				
				# Debug output every 10 iterations
				if self.align_debug_counter % 10 == 0:
					print(f"[ALIGN-DBG] {wheel_name:12} | err={error:7.2f}° | P={pid.last_p_term:7.3f} I={pid.last_i_term:7.3f} D={pid.last_d_term:7.3f} | out={pid_output:7.3f} | spd={speed:7.3f}")
				
				if abs(error) > config.ALIGN_TOLERANCE:
					all_aligned = False
					wheel.set_turn_power(speed)
				else:
					wheel.set_turn_power(0)
			
			self.align_debug_counter += 1
			
			if all_aligned:
				print(f"[ALIGN] All wheels aligned to {self.target_align_angle}°!")
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
		"""Update single-wheel alignment (call every loop)"""
		if not self.aligning_single_wheel or not self.single_wheel_name:
			return
		
		wheel_name = self.single_wheel_name
		target_angle = self.single_wheel_target_angle
		elapsed = wpilib.Timer.getFPGATimestamp() - self.single_wheel_start_time
		
		if wheel_name not in self.wheels:
			print(f"[SINGLE-ALIGN] Unknown wheel: {wheel_name}, stopping")
			self.aligning_single_wheel = False
			return
		
		wheel = self.wheels[wheel_name]
		current_angle = wheel.get_angle()
		
		# Normalize angles to -180 to 180 range for shortest path
		norm_current = current_angle
		if norm_current > 180:
			norm_current -= 360
		
		norm_target = target_angle
		if norm_target > 180:
			norm_target -= 360
		
		# Calculate shortest path error
		error = norm_target - norm_current
		if error > 180:
			error -= 360
		elif error < -180:
			error += 360
		
		# Check if we're close enough (within 2 degrees)
		if abs(error) < 2.0:
			print(f"[SINGLE-ALIGN] {wheel_name} reached target {target_angle}° (current={current_angle:.1f}°)")
			wheel.stop()
			self.aligning_single_wheel = False
			return
		
		# Check timeout (30 seconds per wheel)
		if elapsed > 30:
			print(f"[SINGLE-ALIGN] {wheel_name} timeout after {elapsed:.1f}s, stopping")
			wheel.stop()
			self.aligning_single_wheel = False
			return
		
		# Use PID controller
		pid = self.pid_controllers[wheel_name]
		current_time = wpilib.Timer.getFPGATimestamp()
		pid_output = pid.calculate(error, current_time)
		
		# Clamp output
		speed = max(-config.MOTOR_SCALE_ALIGN, min(config.MOTOR_SCALE_ALIGN, pid_output))
		
		# Boost if too small
		if 0 < abs(speed) < config.MOTOR_SCALE_MANUAL:
			if speed > 0:
				speed = config.MOTOR_SCALE_MANUAL
			else:
				speed = -config.MOTOR_SCALE_MANUAL
		
		# Flip direction (motor wired backward)
		speed = -speed
		
		wheel.set_turn_power(speed)
	
	def start_autotune(self):
		"""Start oscillation-based autotune at 4 angles for all 4 wheels"""
		self.autotuning = True
		print(f"[AUTOTUNE] Starting multi-angle oscillation autotune on all 4 wheels...")
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
		
		# Timeout: 30 seconds per wheel (6 sec × 4 angles + buffer)
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
			print(f"[AUTOTUNE] {wheel_name}: Angle {target_angle}° timeout!")
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
			print(f"[AUTOTUNE] {wheel_name} @ {target_angle}°: KP {old_kp:.6f} -> {current['kp']:.6f}, oscillations={current['sign_changes']}")
		
		# When we detect oscillation (8+ sign changes), record Kc
		if current["sign_changes"] >= 8 and len(current["kc_list"]) == angle_index:
			kc = current["kp"]
			current["kc_list"].append(kc)
			current["osc_start"] = wpilib.Timer.getFPGATimestamp()
			print(f"[AUTOTUNE] {wheel_name} @ {target_angle}°: Found Kc={kc:.6f}, measuring period...")
			current["sign_changes"] = 0  # Reset for period measurement
		
		# Measure period: wait for 8 MORE sign changes after finding Kc
		if len(current["kc_list"]) == angle_index + 1 and len(current["tc_list"]) == angle_index:
			if current["sign_changes"] >= 8:  # 4 full cycles for period
				period_time = wpilib.Timer.getFPGATimestamp() - current["osc_start"]
				tc = period_time / 4.0
				current["tc_list"].append(tc)
				print(f"[AUTOTUNE] {wheel_name} @ {target_angle}°: Tc={tc:.3f}s")
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
			print(f"[AUTOTUNE] {wheel_name}: Moving to {current['angles'][current['angle_index']]}°...\n")
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


