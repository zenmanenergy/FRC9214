import wpilib
from rev import SparkMax, SparkFlex, SparkLowLevel
import CANID


class ShooterSubsystem:
	def __init__(self, turret=None):
		super().__init__()
		
		# Initialize motor controllers
		try:
			self.uptake_motor = SparkMax(CANID.SHOOTER_UPTAKE, SparkLowLevel.MotorType.kBrushless)
		except Exception as e:
			print(f"[SHOOTER] ERROR - Failed to initialize uptake motor (CAN ID {CANID.SHOOTER_UPTAKE}): {type(e).__name__}: {e}", flush=True)
			self.uptake_motor = None
		
		try:
			self.shooter_motor = SparkFlex(CANID.SHOOTER_SHOOTER, SparkLowLevel.MotorType.kBrushless)
		except Exception as e:
			print(f"[SHOOTER] ERROR - Failed to initialize shooter motor (CAN ID {CANID.SHOOTER_SHOOTER}): {type(e).__name__}: {e}", flush=True)
			self.shooter_motor = None
		
		try:
			self.spindexer_motor = SparkMax(CANID.SHOOTER_SPINDEXER, SparkLowLevel.MotorType.kBrushless)
			print(f"[SHOOTER] Spindexer motor initialized on CAN ID {CANID.SHOOTER_SPINDEXER}", flush=True)
		except Exception as e:
			print(f"[SHOOTER] ERROR - Failed to initialize spindexer motor (CAN ID {CANID.SHOOTER_SPINDEXER}): {type(e).__name__}: {e}", flush=True)
			self.spindexer_motor = None
		
		# Turret reference (optional)
		self.turret = turret
		
		# Turret rotation state
		self.target_turret_angle = None
		self.rotating_to_angle = False
		self.rotation_speed = 0.3
		
		# Spindexer state
		self.spindexing = False
		self.spindex_start_time = None
		self.spindex_start_position = None
		self.spindex_state = None  # "ROTATING" or "WAITING"
		self.SPINDEX_ROTATION_TIME = 0.075  # Time to rotate ~90 degrees
		
		# PID autotune state
		self.autotuning = False
	
	def set_uptake(self, speed):
		"""Set uptake motor speed"""
		if self.uptake_motor:
			self.uptake_motor.set(-speed)
	
	def set_turret(self, speed):
		"""Simple turret speed control with hard limits."""
		if not self.turret or abs(speed) < 0.05:
			return
		
		current_angle = self.turret.get_angle()
		left_limit = self.turret.get_left_limit()
		right_limit = self.turret.get_right_limit()
		
		# Always print current state (don't guard with if statement)
		print(f"[TURRET-STATE] Angle: {current_angle:.1f}° | Left: {left_limit}° | Right: {right_limit}° | speed: {speed}", flush=True)
		
		should_block = False
		block_reason = None
		
		# Block only when trying to go PAST the boundaries
		# Negative speed = going right (higher angles), Positive speed = going left (lower angles)
		if current_angle >= right_limit and speed < 0:
			# At/past right limit and trying to go right
			should_block = True
			block_reason = "RIGHT"
		elif current_angle <= left_limit and speed > 0:
			# At/past left limit and trying to go left
			should_block = True
			block_reason = "LEFT"

		
		# Apply speed or block
		if should_block:
			# Only log if state changed
			if not hasattr(self, '_last_block_state') or self._last_block_state != block_reason:
				print(f"[TURRET] BLOCKED {block_reason} | Heading: {current_angle:.1f}° | Limits: {left_limit}° - {right_limit}°", flush=True)
				self._last_block_state = block_reason
			self.turret.set_turn_power(0)
		else:
			self._last_block_state = None
			self.turret.set_turn_power(speed)
					
	
	def rotate_to_angle(self, target_angle, speed=0.3):
		"""
		Rotate turret to target angle, respecting rotation limits.
		Call update_rotation() every cycle to continuously adjust.
		
		Args:
			target_angle: Target angle in degrees (0-360)
			speed: Motor power (-1.0 to 1.0), defaults to 0.3
		"""
		if not self.turret:
			return
		
		# Check if target angle is within limits
		left_limit = self.turret.get_left_limit()
		right_limit = self.turret.get_right_limit()
		current_angle = self.turret.get_angle()
		raw_enc = self.turret.get_raw_encoder_degrees()
		

		
		# Normalize target angle for comparison
		norm_target = target_angle % 360
		
		# Check limit bounds
		is_valid = False
		is_valid = left_limit <= norm_target <= right_limit
		
		if not is_valid:
			print(f"[TURRET] Target angle {norm_target} deg is outside limits ({left_limit} deg to {right_limit} deg). Stopping.", flush=True)
			self.turret.stop()
			self.rotating_to_angle = False
			self.target_turret_angle = None
			return
		
		# Debug: Check if already rotating to same target
		if self.rotating_to_angle and self.target_turret_angle == norm_target:
			return  # Already rotating to this target, don't re-trigger
		
		# Valid target - start rotation
		self.target_turret_angle = norm_target
		self.rotation_speed = speed
		self.rotating_to_angle = True
		print(f"[TURRET] Rotating to {norm_target}° (curr angle: {current_angle}°, raw encoder: {raw_enc:.1f}°, offset: {self.turret.get_zero_offset():.1f}°)", flush=True)
	
	def calculate_pid_power(self, error):
		"""Calculate motor power using PID controller"""
		# Initialize PID state if needed
		if not hasattr(self, '_last_error'):
			self._last_error = error  # Initialize to current error to avoid huge derivative spike
			self._integral_error = 0
		
		# Use tuned gains if available, otherwise use defaults
		if self.use_tuned_gains():
			kp = self.pid_kp
			ki = self.pid_ki
			kd = self.pid_kd
		else:
			# PID gains - derivative disabled to prevent oscillation
			kp = 0.018863  # Increased to tighten control around target
			ki = 0.031178  # Minimal integral gain
			kd = 0.002853  # Disabled


		
		# Calculate integral (accumulate error over time)
		self._integral_error += error * 0.02  # 0.02 is approximate dt (50Hz robot)
		self._integral_error = max(-0.5, min(0.5, self._integral_error))  # Very tight clamp
		
		# Combine P and I terms only
		p_term = kp * error
		i_term = ki * self._integral_error
		motor_power = p_term + i_term
		
		# Debug output
		import time
		if not hasattr(self, '_last_pid_log'):
			self._last_pid_log = 0
		if time.time() - self._last_pid_log > 0.1:
			print(f"[TURRET-PID] Error: {error:.1f}° | P: {p_term:.3f} | I: {i_term:.3f} | Power: {motor_power:.3f}", flush=True)
			self._last_pid_log = time.time()
		
		self._last_error = error
		
		# Negate to match inverted joystick direction (negative = right, positive = left)
		motor_power = -motor_power
		
		# Clamp to motor limits only, not rotation_speed (let PID control smoothly)
		motor_power = max(-1.0, min(1.0, motor_power))
		
		return motor_power
	
	def update_rotation(self):
		"""
		Update turret rotation towards target angle.
		Full speed far away, decelerate when close, stop at target.
		"""
		if not self.rotating_to_angle or not self.target_turret_angle or not self.turret:
			return
		
		current_angle = self.turret.get_angle()
		target_angle = self.target_turret_angle
		
		# Check boundaries
		left_limit = self.turret.get_left_limit()
		right_limit = self.turret.get_right_limit()
		
		# Calculate shortest error path
		error = target_angle - current_angle
		if error > 180:
			error -= 360
		elif error < -180:
			error += 360
		
		# Block movement at limits
		if current_angle <= left_limit and error < 0:
			print(f"[TURRET] Blocked LEFT at {current_angle}°", flush=True)
			self.turret.stop()
			self.rotating_to_angle = False
			return
		elif current_angle >= right_limit and error > 0:
			print(f"[TURRET] Blocked RIGHT at {current_angle}°", flush=True)
			self.turret.stop()
			self.rotating_to_angle = False
			return
		
		# Very close to target - cut power and let brake hold
		if abs(error) < 1.0:
			print(f"[TURRET] Reached target {target_angle}° (actual: {current_angle}°)", flush=True)
			self.turret.stop()  # Cut power - brake engages
			self.rotating_to_angle = False
			return
		
		# Close to target - decelerate with reduced power
		if abs(error) < 3.0:
			if error > 0:
				self.turret.set_turn_power(-0.1)  # Decelerate toward higher angle
			else:
				self.turret.set_turn_power(0.1)   # Decelerate toward lower angle
			return
		
		# Far from target - move at full speed
		if error > 0:
			self.turret.set_turn_power(-self.rotation_speed)  # Target higher = negative power
		else:
			self.turret.set_turn_power(self.rotation_speed)   # Target lower = positive power
	
	def set_shooter(self, speed):
		"""Set shooter motor speed"""
		if self.shooter_motor:
			self.shooter_motor.set(speed)
	
	def spindex(self):
		"""Start continuous spindexer indexing (90 deg rotations with 1 second waits)"""
		if not self.spindexer_motor:
			print("[SHOOTER] ERROR - Spindexer motor not initialized", flush=True)
			return
		
		if self.spindexing:
			print("[SHOOTER] Spindexer already running", flush=True)
			return
		
		import time
		print("[SHOOTER] Spindexer: Starting continuous indexing", flush=True)
		
		# Get encoder and RESET it to 0
		encoder = self.spindexer_motor.getEncoder()
		encoder.setPosition(0.0)  # Reset to 0
		self.spindex_start_position = 0.0
		
		self.spindexing = True
		self.spindex_start_time = time.time()
		self.spindex_state = "ROTATING"
		self.spindexer_motor.set(-0.1)
		print(f"[SHOOTER] Spindexer: Starting rotation, encoder reset to 0", flush=True)
	
	def update_spindex(self):
		"""Update spindexer indexing - track encoder and manage 90 deg rotations with waits"""
		if not self.spindexing or not self.spindexer_motor:
			return
		
		import time
		
		encoder = self.spindexer_motor.getEncoder()
		current_position = encoder.getPosition()
		rotation_delta = abs(current_position - self.spindex_start_position)
		
		if self.spindex_state == "ROTATING":
			# Debug output
			print(f"[SPINDEXER-DEBUG] Start: {self.spindex_start_position:.2f} | Current: {current_position:.2f} | Delta: {rotation_delta:.2f} | Threshold: 0.25", flush=True)
			
			# Check if we've rotated ~90 degrees 
			# Threshold: 0.25 encoder revolutions ~= 90 degrees
			if rotation_delta >= 0.25:
				# Reached 90 degrees - stop and wait
				self.spindexer_motor.set(0.0)
				self.spindex_state = "WAITING"
				self.spindex_wait_start = time.time()
				print(f"[SHOOTER] Spindexer: Rotated {rotation_delta:.2f} ticks, stopping for 1 second", flush=True)
		
		elif self.spindex_state == "WAITING":
			# Wait 1 second before rotating again
			elapsed_wait = time.time() - self.spindex_wait_start
			if elapsed_wait >= 1.0:
				# Time to rotate again - reset encoder and start
				encoder = self.spindexer_motor.getEncoder()
				encoder.setPosition(0.0)  # Reset to 0
				self.spindex_start_position = 0.0
				self.spindex_state = "ROTATING"
				self.spindexer_motor.set(-0.1)
				print(f"[SHOOTER] Spindexer: Starting next rotation, encoder reset to 0", flush=True)
	
	def stop_spindex(self):
		"""Stop the spindexer immediately"""
		if self.spindexer_motor:
			self.spindexer_motor.set(0.0)
		self.spindexing = False
		self.spindex_state = None
		print("[SHOOTER] Spindexer: Stopped", flush=True)
	
	def stop_all(self):
		"""Stop all motors"""
		if self.uptake_motor:
			self.uptake_motor.set(0)
		if self.shooter_motor:
			self.shooter_motor.set(0)
		if self.spindexer_motor:
			self.spindexer_motor.set(0)
		if self.turret:
			self.turret.stop()
		self.rotating_to_angle = False
		self.target_turret_angle = None
		self.spindexing = False
		self.spindex_state = None
	
	def start_pid_autotune(self, center_angle=180):
		"""Start PID autotune using relay oscillation around a safe center point"""
		# Prevent multiple starts
		if self.autotuning:
			print("[AUTOTUNE] Already autotuning! Ignoring restart.", flush=True)
			return
		
		if not self.turret:
			print("[AUTOTUNE] No turret available", flush=True)
			return
		
		# Check limits
		left_limit = self.turret.get_left_limit()
		right_limit = self.turret.get_right_limit()
		
		if not (left_limit <= center_angle <= right_limit):
			print(f"[AUTOTUNE] ERROR: Center angle {center_angle}° outside limits ({left_limit}° - {right_limit}°)", flush=True)
			return
		
		# STOP any existing rotation
		self.rotating_to_angle = False
		self.target_turret_angle = None
		self.turret.stop()
		print("[AUTOTUNE] Stopped any existing rotation", flush=True)
		
		print("[AUTOTUNE] === STARTING PID AUTOTUNE ===", flush=True)
		print(f"[AUTOTUNE] Moving to center angle {center_angle}° first...", flush=True)
		print("[AUTOTUNE] WARNING: DO NOT TOUCH TURRET DURING AUTOTUNE", flush=True)
		
		self.autotuning = True
		self.autotune_center = center_angle
		self.autotune_left_limit = left_limit
		self.autotune_right_limit = right_limit
		self.autotune_power = 0.4  # Relay power for oscillation (bounded by oscillation zone limits)
		self.autotune_start_time = None
		self.autotune_peaks = []  # List of (time, angle) for peaks
		self.autotune_last_angle = self.turret.get_angle()
		self.autotune_max_time = 45  # Max 45 seconds (extra time to move to center)
		self.autotune_phase = "CENTER"  # First phase: move to center
	
	def update_pid_autotune(self):
		"""Update autotune each cycle - first move to center, then oscillate"""
		if not self.autotuning or not self.turret:
			return
		
		import time
		
		current_angle = self.turret.get_angle()
		left_limit = self.turret.get_left_limit()
		right_limit = self.turret.get_right_limit()
		safety_margin = 3  # 3 degree safety margin (smaller than oscillation distance)
		
		# HARD SAFETY: If we're at or past limits, STOP IMMEDIATELY
		if current_angle <= left_limit or current_angle >= right_limit:
			print(f"[AUTOTUNE] EMERGENCY STOP: At limit! angle={current_angle}° limits=({left_limit}°-{right_limit}°)", flush=True)
			self.turret.stop()
			self.autotuning = False
			return
		
		# Initialize start time on first call
		if self.autotune_start_time is None:
			self.autotune_start_time = time.time()
			print(f"[AUTOTUNE] Starting CENTER phase - moving from {current_angle}° to {self.autotune_center}°", flush=True)
		
		elapsed = time.time() - self.autotune_start_time
		
		# Safety: stop after max time
		if elapsed > self.autotune_max_time:
			print(f"[AUTOTUNE] Max time ({self.autotune_max_time}s) reached. Finishing.", flush=True)
			self._finish_pid_autotune()
			return
		
		# PHASE 1: Move to center angle first
		if self.autotune_phase == "CENTER":
			error = self.autotune_center - current_angle
			# Make error wrap around 180
			if error > 180:
				error -= 360
			elif error < -180:
				error += 360
			
			# Check if we can move in this direction without hitting limit
			if error > 0:
				# Trying to move right (higher angles)
				if current_angle + safety_margin >= right_limit:
					# Too close to right limit, stop
					print(f"[AUTOTUNE] Too close to right limit. Stopping at {current_angle}°", flush=True)
					self.turret.stop()
					self.autotuning = False
					return
				move_power = -0.3  # Negative power = clockwise = higher angles
			else:
				# Trying to move left (lower angles)
				if current_angle - safety_margin <= left_limit:
					# Too close to left limit, stop
					print(f"[AUTOTUNE] Too close to left limit. Stopping at {current_angle}°", flush=True)
					self.turret.stop()
					self.autotuning = False
					return
				move_power = 0.3  # Positive power = counter-clockwise = lower angles
			
			if abs(error) < 3:
				# Reached center (within ±3°), switch to oscillation phase
				self.turret.stop()
				print(f"[AUTOTUNE] Reached center angle {current_angle}°. Starting oscillation phase...", flush=True)
				self.autotune_phase = "OSCILLATE"
				self.autotune_oscillate_start = time.time()
				self.autotune_peaks = []
				self._last_relay_power = None
				return
			else:
				# Move toward center
				self.turret.set_turn_power(move_power)
				print(f"[AUTOTUNE] CENTER: angle={current_angle:.1f}° error={error:.1f}° power={move_power}", flush=True)
				return
		
		# PHASE 2: Oscillate around center (boundary-based reversal, not time-based)
		if self.autotune_phase == "OSCILLATE":
			oscillate_elapsed = time.time() - self.autotune_oscillate_start
			
			# Define safe oscillation bounds (±10° from center)
			osc_min = self.autotune_center - 10
			osc_max = self.autotune_center + 10
			
			# Initialize oscillation power direction if needed
			if not hasattr(self, '_oscillation_power'):
				self._oscillation_power = self.autotune_power  # Start with positive
				self._last_osc_angle = current_angle
				print(f"[AUTOTUNE] Starting oscillation with power={self._oscillation_power}", flush=True)
			
			# Safety checks - hard limits
			if current_angle <= self.autotune_left_limit + 5:
				print(f"[AUTOTUNE] EMERGENCY: Hit left hard limit at {current_angle}°", flush=True)
				self._finish_pid_autotune()
				return
			elif current_angle >= self.autotune_right_limit - 5:
				print(f"[AUTOTUNE] EMERGENCY: Hit right hard limit at {current_angle}°", flush=True)
				self._finish_pid_autotune()
				return
			
			# Check if we crossed a boundary - if so, reverse power
			crossed_boundary = False
			if current_angle <= osc_min and self._oscillation_power > 0:
				# Hit lower bound while moving down (positive power), reverse to negative
				self._oscillation_power = -self.autotune_power
				crossed_boundary = True
				print(f"[AUTOTUNE] Hit lower bound at {current_angle}°, reversing to negative power", flush=True)
			elif current_angle >= osc_max and self._oscillation_power < 0:
				# Hit upper bound while moving up (negative power), reverse to positive
				self._oscillation_power = self.autotune_power
				crossed_boundary = True
				print(f"[AUTOTUNE] Hit upper bound at {current_angle}°, reversing to positive power", flush=True)
			
			# Record peak when power reverses
			if crossed_boundary:
				self.autotune_peaks.append((oscillate_elapsed, current_angle))
				print(f"[AUTOTUNE] Peak {len(self.autotune_peaks)}: {current_angle}° at {oscillate_elapsed:.2f}s", flush=True)
			
			# Apply current oscillation power
			self.turret.set_turn_power(self._oscillation_power)
			
			# Debug output every 2 seconds
			if not hasattr(self, '_last_autotune_debug'):
				self._last_autotune_debug = 0
			if oscillate_elapsed - self._last_autotune_debug > 2.0:
				print(f"[AUTOTUNE] Elapsed: {oscillate_elapsed:.1f}s | Angle: {current_angle}° (zone: {osc_min}°-{osc_max}°) | Power: {self._oscillation_power:.2f} | Peaks: {len(self.autotune_peaks)}", flush=True)
				self._last_autotune_debug = oscillate_elapsed
			
			# Stop after enough peaks detected
			if len(self.autotune_peaks) >= 6:
				print(f"[AUTOTUNE] Got {len(self.autotune_peaks)} peaks. Finishing.", flush=True)
				self._finish_pid_autotune()
	
	def _finish_pid_autotune(self):
		"""Calculate gains and apply them"""
		import time
		
		self.autotuning = False
		self.turret.stop()
		
		if len(self.autotune_peaks) < 2:
			print("[AUTOTUNE] ERROR: Not enough peaks detected. Cancelling autotune.", flush=True)
			return
		
		# Calculate oscillation period (time between same type of peaks)
		peak_times = [p[0] for p in self.autotune_peaks]
		periods = []
		for i in range(len(peak_times) - 2):
			period = peak_times[i + 2] - peak_times[i]  # Full cycle is 2 peaks apart
			periods.append(period)
		
		if not periods:
			print("[AUTOTUNE] ERROR: Could not calculate period. Cancelling autotune.", flush=True)
			return
		
		Tu = sum(periods) / len(periods)  # Average period
		
		# For relay oscillation: Ku = (4 * A) / (π * B)
		# where A = relay amplitude, B = oscillation amplitude
		peak_angles = [p[1] for p in self.autotune_peaks]
		osc_amplitude = (max(peak_angles) - min(peak_angles)) / 2.0
		relay_amplitude = self.autotune_power
		
		if osc_amplitude < 0.5:
			print("[AUTOTUNE] ERROR: Oscillation amplitude too small. Cancelling autotune.", flush=True)
			return
		
		Ku = (4 * relay_amplitude) / (3.14159 * osc_amplitude)
		
		# Ziegler-Nichols tuning (classic PID)
		# For some overshoot: kp = 0.6*Ku, ki = 1.2*Ku/Tu, kd = 0.075*Ku*Tu
		# More conservative (less overshoot): kp = 0.5*Ku, ki = 0.5*Ku/Tu, kd = 0.125*Ku*Tu
		
		kp_new = 0.5 * Ku
		ki_new = 0.5 * Ku / Tu if Tu > 0 else 0.0001
		kd_new = 0.125 * Ku * Tu
		
		print(f"[AUTOTUNE] === TUNING COMPLETE ===", flush=True)
		print(f"[AUTOTUNE] Ku (critical gain): {Ku:.4f}", flush=True)
		print(f"[AUTOTUNE] Tu (period): {Tu:.4f}s", flush=True)
		print(f"[AUTOTUNE] Oscillation amplitude: {osc_amplitude:.2f}°", flush=True)
		print(f"[AUTOTUNE] === CALCULATED GAINS ===", flush=True)
		print(f"[AUTOTUNE] kp = {kp_new:.6f}", flush=True)
		print(f"[AUTOTUNE] ki = {ki_new:.6f}", flush=True)
		print(f"[AUTOTUNE] kd = {kd_new:.6f}", flush=True)
		
		# Store new gains (they'll be used in calculate_pid_power)
		self.autotune_kp = kp_new
		self.autotune_ki = ki_new
		self.autotune_kd = kd_new
		
		print("[AUTOTUNE] New gains ready. Call apply_autotune_gains() to use them.", flush=True)
	
	def apply_autotune_gains(self):
		"""Apply the autotuned gains"""
		if not hasattr(self, 'autotune_kp'):
			print("[AUTOTUNE] No tuned gains available. Run start_pid_autotune() first.", flush=True)
			return
		
		print(f"[AUTOTUNE] Applying gains: kp={self.autotune_kp:.6f}, ki={self.autotune_ki:.6f}, kd={self.autotune_kd:.6f}", flush=True)
		self.pid_kp = self.autotune_kp
		self.pid_ki = self.autotune_ki
		self.pid_kd = self.autotune_kd
		print("[AUTOTUNE] Gains applied!", flush=True)
	
	def use_tuned_gains(self):
		"""Check if we should use tuned gains in calculate_pid_power"""
		return hasattr(self, 'pid_kp')
