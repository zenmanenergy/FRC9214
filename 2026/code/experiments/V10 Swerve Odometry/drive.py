import math
import json
import os
try:
	import rev
	HAS_REV = True
except (ImportError, ModuleNotFoundError):
	HAS_REV = False
	rev = None

try:
	from wpilib import Encoder
	HAS_WPILIB = True
except (ImportError, ModuleNotFoundError):
	HAS_WPILIB = False
	Encoder = None


class SwerveModule:
	"""A single swerve module with drive and turn motors."""

	def __init__(self, drive_motor_id: int, turn_motor_id: int, encoder_channel_a: int = None, encoder_channel_b: int = None, name: str = ""):
		"""Initialize a swerve module with REV SparkMax motors and encoder."""
		self.drive_motor_id = drive_motor_id
		self.turn_motor_id = turn_motor_id
		self.drive_motor = None
		self.turn_motor = None
		self.encoder = None
		self.name = name
		self.drive_power_scale = 1.0  # Per-motor power scaling factor
		
		if not HAS_REV:
			return

		self.drive_motor = rev.SparkMax(drive_motor_id, rev.SparkMax.MotorType.kBrushless)
		self.turn_motor = rev.SparkMax(turn_motor_id, rev.SparkMax.MotorType.kBrushless)
		
		# Initialize encoder if DIO ports are provided
		if encoder_channel_a is not None and encoder_channel_b is not None and HAS_WPILIB:
			self.encoder = Encoder(encoder_channel_a, encoder_channel_b)
			self.encoder.setDistancePerPulse(1.0)  # 1 pulse per count
		
		print("M%d INIT: Drive=%d Turn=%d" % (drive_motor_id, drive_motor_id, turn_motor_id))

	def set_drive(self, speed: float):
		"""Set drive motor speed."""
		if self.drive_motor is None:
			return
		self.drive_motor.set(speed)

	def set_turn(self, speed: float):
		"""Set turn motor speed."""
		if self.turn_motor is None:
			return
		self.turn_motor.set(speed)

	def stop(self):
		"""Stop both motors."""
		if self.drive_motor is not None:
			self.drive_motor.set(0.0)
		if self.turn_motor is not None:
			self.turn_motor.set(0.0)

	def get_encoder_count(self) -> int:
		"""Get encoder count."""
		if self.encoder is None:
			return 0
		return self.encoder.get()

	def get_encoder_distance(self) -> float:
		"""Get encoder distance."""
		if self.encoder is None:
			return 0.0
		return self.encoder.getDistance()




class SwerveDrive:
	"""Controls a 4-wheel swerve drive."""

	def __init__(
		self,
		front_left: SwerveModule,
		front_right: SwerveModule,
		rear_left: SwerveModule,
		rear_right: SwerveModule,
	):
		"""Initialize the swerve drive."""
		self.front_left = front_left
		self.front_right = front_right
		self.rear_left = rear_left
		self.rear_right = rear_right
		
		# Drive motor PID gains [FL, FR, RL, RR]
		self.kP = [0.02, 0.02, 0.02, 0.02]  # Proportional per motor
		self.kI = [0.003, 0.003, 0.003, 0.003]  # Integral per motor
		self.kD = [0.01, 0.01, 0.01, 0.01]  # Derivative per motor
		
		# Turn motor PID gains [FL, FR, RL, RR]
		self.kP_turn = [0.02, 0.02, 0.02, 0.02]  # Turn proportional
		self.kI_turn = [0.003, 0.003, 0.003, 0.003]  # Turn integral
		self.kD_turn = [0.01, 0.01, 0.01, 0.01]  # Turn derivative
		
		self.last_encoder_counts = [0, 0, 0, 0]
		self.accumulated_error = [0, 0, 0, 0]  # For integral term
		self.last_error = [0, 0, 0, 0]  # For derivative term
		
		# Autotune state tracking
		self.tuning_active = False
		self.tuning_phase = None  # "drive" or "turn"
		self.tuning_start_time = None
		self.last_direction_time = None
		self.relay_direction = [1, 1, 1, 1]
		self.peak_counts = [[], [], [], []]
		self.initial_counts = [0, 0, 0, 0]
		self.tuning_results = {}  # Store results from both phases

	def drive(self, forward_speed: float, rotation_speed: float):
		"""Drive with forward/backward and rotation using encoder-based speed correction."""
		drive_speed = forward_speed * 0.3
		turn_speed = rotation_speed * 0.3
		
		# Get current encoder counts
		fl_count = self.front_left.get_encoder_count()
		fr_count = self.front_right.get_encoder_count()
		rl_count = self.rear_left.get_encoder_count()
		rr_count = self.rear_right.get_encoder_count()
		
		# Calculate encoder deltas
		fl_delta = fl_count - self.last_encoder_counts[0]
		fr_delta = fr_count - self.last_encoder_counts[1]
		rl_delta = rl_count - self.last_encoder_counts[2]
		rr_delta = rr_count - self.last_encoder_counts[3]
		
		self.last_encoder_counts = [fl_count, fr_count, rl_count, rr_count]
		
		# Debug counter - only print every 10th cycle to reduce spam
		self.debug_counter = getattr(self, 'debug_counter', 0) + 1
		should_debug = self.debug_counter % 10 == 0
		
		# Track mode transitions to reset accumulated error when switching between rotation and forward
		if not hasattr(self, 'in_rotation_mode'):
			self.in_rotation_mode = False
		if not hasattr(self, 'rotation_exit_counter'):
			self.rotation_exit_counter = 0
		
		# If rotation is significant, do rotate-in-place with cross pattern + PID for precision
		if abs(rotation_speed) > 0.1:
			self.in_rotation_mode = True  # Mark that we're in rotation mode
			# Detect rotation direction change - reset accumulated error on reversal
			if not hasattr(self, 'last_rotation_speed'):
				self.last_rotation_speed = 0
			if (self.last_rotation_speed > 0.1 and rotation_speed < -0.1) or (self.last_rotation_speed < -0.1 and rotation_speed > 0.1):
				# Rotation direction reversed - reset error to prevent windup accumulation
				self.accumulated_error = [0, 0, 0, 0]
				self.last_error = [0, 0, 0, 0]
			self.last_rotation_speed = rotation_speed
			
			# Rotate in place: angle wheels and spin in cross pattern
			self.front_left.set_turn(-turn_speed)
			self.front_right.set_turn(-turn_speed)
			self.rear_left.set_turn(-turn_speed)
			self.rear_right.set_turn(-turn_speed)
			
			# Cross pattern for rotation with PID feedback for smooth control
			rotation_drive = rotation_speed * 0.6
			fl_base = rotation_drive * self.front_left.drive_power_scale
			fr_base = -rotation_drive * self.front_right.drive_power_scale
			rl_base = -rotation_drive * self.rear_left.drive_power_scale
			rr_base = rotation_drive * self.rear_right.drive_power_scale
			
			# Calculate encoder deltas for rotation sync error (FL+RR vs FR+RL pairs)
			# FL and RR should move in same direction at same speed
			fl_rr_avg = (fl_delta + rr_delta) / 2.0
			# FR and RL should move in same direction at same speed  
			fr_rl_avg = (fr_delta + rl_delta) / 2.0
			
			# Errors: how much each deviates from its pair's average
			rotation_errors = [
				fl_rr_avg - fl_delta,  # FL vs FL+RR pair
				fr_rl_avg - fr_delta,  # FR vs FR+RL pair
				fr_rl_avg - rl_delta,  # RL vs FR+RL pair
				fl_rr_avg - rr_delta   # RR vs FL+RR pair
			]
			
			# Apply deadband to rotation errors like forward driving (ignore encoder jitter)
			error_deadband = 1.2
			rotation_errors = [e if abs(e) > error_deadband else 0 for e in rotation_errors]
			
			# Update accumulated errors with TIGHT clamping to prevent windup
			for i in range(4):
				self.accumulated_error[i] += rotation_errors[i]
				# Very tight clamping for rotation (±5 to prevent integral windup with varying motor gains)
				self.accumulated_error[i] = max(-5, min(5, self.accumulated_error[i]))
			
			# Very gentle damping for rotation to prevent aggressive overcorrection
			rotation_damping = 0.05
			
			# Calculate PID corrections using per-motor drive gains (turn gains are for angle only)
			p = rotation_errors[0] * self.kP[0]
			i = self.accumulated_error[0] * self.kI[0]
			d = (rotation_errors[0] - self.last_error[0]) * self.kD[0]
			fl_correction = (p + i + d) * rotation_damping
			
			p = rotation_errors[1] * self.kP[1]
			i = self.accumulated_error[1] * self.kI[1]
			d = (rotation_errors[1] - self.last_error[1]) * self.kD[1]
			fr_correction = (p + i + d) * rotation_damping
			
			p = rotation_errors[2] * self.kP[2]
			i = self.accumulated_error[2] * self.kI[2]
			d = (rotation_errors[2] - self.last_error[2]) * self.kD[2]
			rl_correction = (p + i + d) * rotation_damping
			
			p = rotation_errors[3] * self.kP[3]
			i = self.accumulated_error[3] * self.kI[3]
			d = (rotation_errors[3] - self.last_error[3]) * self.kD[3]
			rr_correction = (p + i + d) * rotation_damping
			
			# Store errors for next iteration
			self.last_error = rotation_errors
			
			# Apply base commands + PID corrections, clamped to ±0.9 to prevent motor saturation
			fl_cmd = max(-0.9, min(0.9, fl_base + fl_correction))
			fr_cmd = max(-0.9, min(0.9, fr_base + fr_correction))
			rl_cmd = max(-0.9, min(0.9, rl_base + rl_correction))
			rr_cmd = max(-0.9, min(0.9, rr_base + rr_correction))
			self.front_left.set_drive(fl_cmd)
			self.front_right.set_drive(fr_cmd)
			self.rear_left.set_drive(rl_cmd)
			self.rear_right.set_drive(rr_cmd)
			
			if should_debug and abs(rotation_speed) > 0.05:
				print(f"[ROTATE] Cmd: FL:{fl_cmd:+.3f} FR:{fr_cmd:+.3f} RL:{rl_cmd:+.3f} RR:{rr_cmd:+.3f} | Err: FL:{rotation_errors[0]:+.1f} FR:{rotation_errors[1]:+.1f} RL:{rotation_errors[2]:+.1f} RR:{rotation_errors[3]:+.1f} | AccumErr: {[round(e, 2) for e in self.accumulated_error]}")
		else:
			# Exiting rotation mode - reset accumulated error and initiate wheel transition
			if self.in_rotation_mode:
				self.accumulated_error = [0, 0, 0, 0]
				self.last_error = [0, 0, 0, 0]
				self.in_rotation_mode = False
				self.rotation_exit_counter = 10  # Give wheels 10 cycles for smoother transition
			
			# If transitioning from rotation, apply gentle turn correction to bring wheels to 0°
			if self.rotation_exit_counter > 0:
				self.front_left.set_turn(0.1)  # Gentle wheels toward 0° during transition
				self.front_right.set_turn(0.1)
				self.rear_left.set_turn(0.1)
				self.rear_right.set_turn(0.1)
				self.rotation_exit_counter -= 1
			else:
				# Normal forward mode - apply gentle turn correction
				self.front_left.set_turn(-turn_speed * 0.1)
				self.front_right.set_turn(-turn_speed * 0.1)
				self.rear_left.set_turn(-turn_speed * 0.1)
				self.rear_right.set_turn(-turn_speed * 0.1)
			
			# Forward driving mode with power scaling baseline + PID control
			fl_base = drive_speed * self.front_left.drive_power_scale
			fr_base = drive_speed * self.front_right.drive_power_scale
			rl_base = drive_speed * self.rear_left.drive_power_scale
			rr_base = drive_speed * self.rear_right.drive_power_scale
			
			avg_delta = (fl_delta + fr_delta + rl_delta + rr_delta) / 4.0
			
			# Calculate errors for each wheel with deadband to ignore encoder jitter
			error_deadband = 1.2  # Ignore errors smaller than 1.2 encoder counts
			errors = [
				avg_delta - fl_delta if abs(avg_delta - fl_delta) > error_deadband else 0,
				avg_delta - fr_delta if abs(avg_delta - fr_delta) > error_deadband else 0,
				avg_delta - rl_delta if abs(avg_delta - rl_delta) > error_deadband else 0,
				avg_delta - rr_delta if abs(avg_delta - rr_delta) > error_deadband else 0
			]
			
			# Update accumulated errors (integral term) with clamping
			for i in range(4):
				self.accumulated_error[i] += errors[i]
				self.accumulated_error[i] = max(-100, min(100, self.accumulated_error[i]))  # Tighter clamping
			
			# Use much gentler damping for forward driving (0.2 instead of 0.5)
			forward_damping = 0.2
			
			# Calculate PID corrections using per-motor gains
			p = errors[0] * self.kP[0]
			i = self.accumulated_error[0] * self.kI[0]
			d = (errors[0] - self.last_error[0]) * self.kD[0]
			fl_correction = (p + i + d) * forward_damping
			
			p = errors[1] * self.kP[1]
			i = self.accumulated_error[1] * self.kI[1]
			d = (errors[1] - self.last_error[1]) * self.kD[1]
			fr_correction = (p + i + d) * forward_damping
			
			p = errors[2] * self.kP[2]
			i = self.accumulated_error[2] * self.kI[2]
			d = (errors[2] - self.last_error[2]) * self.kD[2]
			rl_correction = (p + i + d) * forward_damping
			
			p = errors[3] * self.kP[3]
			i = self.accumulated_error[3] * self.kI[3]
			d = (errors[3] - self.last_error[3]) * self.kD[3]
			rr_correction = (p + i + d) * forward_damping
			
			# Store errors for next iteration
			self.last_error = errors
			
			# Apply scaled base + PID corrections, clamped to ±0.9 to prevent motor saturation
			fl_cmd = max(-0.9, min(0.9, fl_base + fl_correction))
			fr_cmd = max(-0.9, min(0.9, fr_base + fr_correction))
			rl_cmd = max(-0.9, min(0.9, rl_base + rl_correction))
			rr_cmd = max(-0.9, min(0.9, rr_base + rr_correction))
			
			# During wheel transition (after rotation), reduce drive commands to avoid conflicting kinematics
			if self.rotation_exit_counter > 0:
				# Scale down commands during transition for smoother wheel rotation
				scale_factor = self.rotation_exit_counter / 10.0  # Gradually fade from 0.0 to 1.0
				fl_cmd *= (1.0 - scale_factor)
				fr_cmd *= (1.0 - scale_factor)
				rl_cmd *= (1.0 - scale_factor)
				rr_cmd *= (1.0 - scale_factor)
			
			self.front_left.set_drive(fl_cmd)
			self.front_right.set_drive(fr_cmd)
			self.rear_left.set_drive(rl_cmd)
			self.rear_right.set_drive(rr_cmd)
			
			# Debug output for forward driving
			if abs(forward_speed) > 0.05:
				print(f"[DRIVE] Cmd: FL:{fl_cmd:+.3f} FR:{fr_cmd:+.3f} RL:{rl_cmd:+.3f} RR:{rr_cmd:+.3f} | Err: FL:{errors[0]:+.1f} FR:{errors[1]:+.1f} RL:{errors[2]:+.1f} RR:{errors[3]:+.1f}" + (f" [TRANSITION]" if self.rotation_exit_counter > 0 else ""))

	def stop(self):
		"""Stop all motors."""
		self.front_left.stop()
		self.front_right.stop()
		self.rear_left.stop()
		self.rear_right.stop()
		# Reset accumulated error to prevent windup when stopped
		self.accumulated_error = [0, 0, 0, 0]
		self.last_error = [0, 0, 0, 0]

	def autotune_pid(self):
		"""Start non-blocking PID auto-tuning for BOTH drive and turn motors (20 seconds total). Call update_autotune_pid each cycle."""
		print("\n===== PID AUTO-TUNING STARTING (20 seconds: 10s drive + 10s turn) =====")
		print("[DEBUG] Monitoring motor responses during relay switching...\n")
		
		import time
		# Initialize tuning state - start with DRIVE phase
		self.tuning_active = True
		self.tuning_phase = "drive"
		self.tuning_start_time = time.time()
		self.last_direction_time = time.time()
		self.relay_direction = [1, 1, 1, 1]
		self.peak_counts = [[], [], [], []]
		self.initial_counts = [
			self.front_left.get_encoder_count(),
			self.front_right.get_encoder_count(),
			self.rear_left.get_encoder_count(),
			self.rear_right.get_encoder_count()
		]
		self.tuning_results = {}
		print(f"[PHASE] Starting DRIVE motor tuning...")
		print(f"[DEBUG] Initial encoder counts: FL={self.initial_counts[0]} FR={self.initial_counts[1]} RL={self.initial_counts[2]} RR={self.initial_counts[3]}")
	
	def update_autotune_pid(self):
		"""Update autotune state - call this each teleopPeriodic. Returns (gains, is_complete) or (None, False)."""
		if not self.tuning_active:
			return None, False
		
		import time
		switch_interval = 0.5
		relay_power = 0.4
		tuning_duration = 10.0
		
		# Send current relay commands based on phase
		for i in range(4):
			cmd = self.relay_direction[i] * relay_power
			if self.tuning_phase == "drive":
				if i == 0:
					self.front_left.set_drive(cmd)
				elif i == 1:
					self.front_right.set_drive(cmd)
				elif i == 2:
					self.rear_left.set_drive(cmd)
				else:
					self.rear_right.set_drive(cmd)
			else:  # turn phase
				if i == 0:
					self.front_left.set_turn(cmd)
				elif i == 1:
					self.front_right.set_turn(cmd)
				elif i == 2:
					self.rear_left.set_turn(cmd)
				else:
					self.rear_right.set_turn(cmd)
		
		# Get current encoder values
		current_counts = [
			self.front_left.get_encoder_count(),
			self.front_right.get_encoder_count(),
			self.rear_left.get_encoder_count(),
			self.rear_right.get_encoder_count()
		]
		names = ["FL", "FR", "RL", "RR"]
		direction_symbols = ["+" if d > 0 else "-" for d in self.relay_direction]
		phase_label = "DRIVE" if self.tuning_phase == "drive" else "TURN"
		print(f"[{phase_label}] Dir: {direction_symbols[0]}{direction_symbols[1]}{direction_symbols[2]}{direction_symbols[3]} | Cmd: {relay_power:.1f} | Counts: FL={current_counts[0]:6d} FR={current_counts[1]:6d} RL={current_counts[2]:6d} RR={current_counts[3]:6d}")
		
		# Check if we should switch direction
		if time.time() - self.last_direction_time > switch_interval:
			for i in range(4):
				self.peak_counts[i].append(current_counts[i])
				self.relay_direction[i] *= -1
			self.last_direction_time = time.time()
			print(f"[{phase_label}] *** DIRECTION SWITCHED ***\n")
		
		# Check if current phase is complete
		if time.time() - self.tuning_start_time >= tuning_duration:
			# Finalize current phase
			phase_gains = self._finalize_phase()
			self.tuning_results[self.tuning_phase] = phase_gains
			
			# Check if we should start turn phase
			if self.tuning_phase == "drive":
				print("\n[PHASE] Switching to TURN motor tuning...")
				self.tuning_phase = "turn"
				self.tuning_start_time = time.time()
				self.last_direction_time = time.time()
				self.relay_direction = [1, 1, 1, 1]
				self.peak_counts = [[], [], [], []]
				return None, False
			else:  # turn phase complete
				return self._finalize_complete(), True
		
		return None, False
	
	def _finalize_phase(self):
		
		# Stop motors (but keep tuning active for next phase)
		self.stop()
		
		# Calculate tuning parameters from amplitude and frequency
		print("\n===== TUNING RESULTS =====")
		amplitudes = []
		frequencies = []
		
		switch_interval = 0.5  # Must match what's in update_autotune_pid
		relay_power = 0.4  # Must match what's in update_autotune_pid
		names = ["FL", "FR", "RL", "RR"]
		for i in range(4):
			if len(self.peak_counts[i]) >= 3:
				# Calculate differences between consecutive peak counts
				diffs = []
				for j in range(len(self.peak_counts[i]) - 1):
					diff = abs(self.peak_counts[i][j+1] - self.peak_counts[i][j])
					diffs.append(diff)
				avg_amplitude = sum(diffs) / len(diffs) if diffs else 1.0
				amplitudes.append(avg_amplitude)
				
				# Period is 2x switch_interval (full cycle)
				freq = 1.0 / (2.0 * switch_interval)
				frequencies.append(freq)
				
				print(f"[RESULT] {names[i]}: Peak counts={self.peak_counts[i]} | Amplitude={avg_amplitude:.1f}, Freq={freq:.2f}Hz")
			else:
				amplitudes.append(1.0)
				frequencies.append(1.0)
				print(f"[RESULT] {names[i]}: Only {len(self.peak_counts[i])} peaks recorded, using defaults")
		
		# Calculate per-motor gains using individual amplitudes and frequencies
		print(f"\n===== CALCULATED PER-MOTOR GAINS =====")
		
		kp_list = []
		ki_list = []
		kd_list = []
		names = ["FL", "FR", "RL", "RR"]
		
		for i in range(4):
			# Use individual amplitude and frequency for each motor
			motor_amplitude = amplitudes[i]
			motor_frequency = frequencies[i]
			
			# Simplified Ziegler-Nichols
			ku = relay_power / (0.5 * motor_amplitude) if motor_amplitude > 0 else 1.0
			pu = 1.0 / motor_frequency if motor_frequency > 0 else 1.0
			
			# Classic PID tuning
			kp = 0.6 * ku
			ki = 1.2 * ku / pu if pu > 0 else 0.001
			kd = 0.075 * ku * pu
			
			kp_list.append(kp)
			ki_list.append(ki)
			kd_list.append(kd)
			
			print(f"{names[i]}: Amplitude={motor_amplitude:.2f} | Ku={ku:.6f} | kP={kp:.6f}, kI={ki:.8f}, kD={kd:.6f}")
		
		print("========================================")
		print(f"[PHASE] {self.tuning_phase.upper()} motor tuning complete\n")
		
		return kp_list, ki_list, kd_list
	
	def _finalize_complete(self):
		"""Finalize both phases, save all gains, and return them."""
		# Stop motors
		self.stop()
		self.tuning_active = False
		
		# Extract results from both phases
		drive_kp, drive_ki, drive_kd = self.tuning_results.get("drive", ([0]*4, [0]*4, [0]*4))
		turn_kp, turn_ki, turn_kd = self.tuning_results.get("turn", ([0]*4, [0]*4, [0]*4))
		
		# Print final summary
		print("\n===== AUTOTUNE COMPLETE - ALL PHASES FINISHED =====")
		names = ["FL", "FR", "RL", "RR"]
		
		print("\nDRIVE MOTORS:")
		for i in range(4):
			print(f"{names[i]}: kP={drive_kp[i]:.6f} kI={drive_ki[i]:.8f} kD={drive_kd[i]:.6f}")
		
		print("\nTURN MOTORS:")
		for i in range(4):
			print(f"{names[i]}: kP={turn_kp[i]:.6f} kI={turn_ki[i]:.8f} kD={turn_kd[i]:.6f}")
		
		print("========================================")
		
		# Save all gains to file
		self.save_pid_gains_complete(drive_kp, drive_ki, drive_kd, turn_kp, turn_ki, turn_kd)
		
		print("[SUCCESS] All PID data has been saved!")
		print("========================================\n")
		
		# Return combined gains tuple (drive gains first, then turn gains)
		return ((drive_kp, drive_ki, drive_kd), (turn_kp, turn_ki, turn_kd))
	
	def save_pid_gains_complete(self, drive_kp, drive_ki, drive_kd, turn_kp, turn_ki, turn_kd):
		"""Save both drive and turn PID gains to JSON file on RoboRIO."""
		try:
			filepath = "/home/lvuser/frc9214_swerve_pid_gains.json"
			
			gains_data = {
				"drive": {
					"FL": {"kP": drive_kp[0], "kI": drive_ki[0], "kD": drive_kd[0]},
					"FR": {"kP": drive_kp[1], "kI": drive_ki[1], "kD": drive_kd[1]},
					"RL": {"kP": drive_kp[2], "kI": drive_ki[2], "kD": drive_kd[2]},
					"RR": {"kP": drive_kp[3], "kI": drive_ki[3], "kD": drive_kd[3]},
				},
				"turn": {
					"FL": {"kP": turn_kp[0], "kI": turn_ki[0], "kD": turn_kd[0]},
					"FR": {"kP": turn_kp[1], "kI": turn_ki[1], "kD": turn_kd[1]},
					"RL": {"kP": turn_kp[2], "kI": turn_ki[2], "kD": turn_kd[2]},
					"RR": {"kP": turn_kp[3], "kI": turn_ki[3], "kD": turn_kd[3]},
				}
			}
			
			with open(filepath, "w") as f:
				json.dump(gains_data, f, indent=2)
			
			print(f"[SAVED] All PID gains saved to {filepath}")
			return True
		except Exception as e:
			print(f"[ERROR] Failed to save PID gains: {e}")
			return False
	
	def save_pid_gains(self, kp_list, ki_list, kd_list):
		"""Save per-motor PID gains to JSON file on RoboRIO."""
		try:
			filepath = "/home/lvuser/frc9214_swerve_pid_gains.json"
			
			gains_data = {
				"FL": {"kP": kp_list[0], "kI": ki_list[0], "kD": kd_list[0]},
				"FR": {"kP": kp_list[1], "kI": ki_list[1], "kD": kd_list[1]},
				"RL": {"kP": kp_list[2], "kI": ki_list[2], "kD": kd_list[2]},
				"RR": {"kP": kp_list[3], "kI": ki_list[3], "kD": kd_list[3]},
			}
			
			with open(filepath, "w") as f:
				json.dump(gains_data, f, indent=2)
			
			print(f"[SAVED] PID gains saved to {filepath}")
			return True
		except Exception as e:
			print(f"[ERROR] Failed to save PID gains: {e}")
			return False
	
	def load_pid_gains(self):
		"""Load per-motor PID gains from JSON file on RoboRIO (handles both old and new formats)."""
		try:
			filepath = "/home/lvuser/frc9214_swerve_pid_gains.json"
			
			if not os.path.exists(filepath):
				print(f"[INFO] No saved gains found at {filepath}, using defaults")
				return False
			
			with open(filepath, "r") as f:
				gains_data = json.load(f)
			
			# Debug: print raw JSON data
			print(f"\n[DEBUG] Raw JSON loaded from {filepath}:")
			print(f"[DEBUG] {json.dumps(gains_data, indent=2)}\n")
			
			names = ["FL", "FR", "RL", "RR"]
			
			# Check if new format (with drive/turn sections)
			if "drive" in gains_data and "turn" in gains_data:
				# New nested format
				drive_data = gains_data["drive"]
				turn_data = gains_data["turn"]
				
				# Debug: print drive and turn data sections
				print("[DEBUG] DRIVE DATA FROM FILE:")
				for name in names:
					if name in drive_data:
						print(f"[DEBUG]   {name}: {drive_data[name]}")
				print("[DEBUG] TURN DATA FROM FILE:")
				for name in names:
					if name in turn_data:
						print(f"[DEBUG]   {name}: {turn_data[name]}\n")
				
				# Load drive motor gains
				self.kP = [
					drive_data["FL"].get("kP", self.kP[0]),
					drive_data["FR"].get("kP", self.kP[1]),
					drive_data["RL"].get("kP", self.kP[2]),
					drive_data["RR"].get("kP", self.kP[3]),
				]
				self.kI = [
					drive_data["FL"].get("kI", self.kI[0]),
					drive_data["FR"].get("kI", self.kI[1]),
					drive_data["RL"].get("kI", self.kI[2]),
					drive_data["RR"].get("kI", self.kI[3]),
				]
				self.kD = [
					drive_data["FL"].get("kD", self.kD[0]),
					drive_data["FR"].get("kD", self.kD[1]),
					drive_data["RL"].get("kD", self.kD[2]),
					drive_data["RR"].get("kD", self.kD[3]),
				]
				
				# Load turn motor gains
				self.kP_turn = [
					turn_data["FL"].get("kP", self.kP_turn[0]),
					turn_data["FR"].get("kP", self.kP_turn[1]),
					turn_data["RL"].get("kP", self.kP_turn[2]),
					turn_data["RR"].get("kP", self.kP_turn[3]),
				]
				self.kI_turn = [
					turn_data["FL"].get("kI", self.kI_turn[0]),
					turn_data["FR"].get("kI", self.kI_turn[1]),
					turn_data["RL"].get("kI", self.kI_turn[2]),
					turn_data["RR"].get("kI", self.kI_turn[3]),
				]
				self.kD_turn = [
					turn_data["FL"].get("kD", self.kD_turn[0]),
					turn_data["FR"].get("kD", self.kD_turn[1]),
					turn_data["RL"].get("kD", self.kD_turn[2]),
					turn_data["RR"].get("kD", self.kD_turn[3]),
				]
				
				print(f"[LOADED] Drive & Turn PID gains loaded from {filepath}")
				print("[LOADED] DRIVE MOTORS:")
				for i in range(4):
					print(f"[LOADED]   {names[i]}: kP={self.kP[i]:.6f} kI={self.kI[i]:.8f} kD={self.kD[i]:.6f}")
				print("[LOADED] TURN MOTORS:")
				for i in range(4):
					print(f"[LOADED]   {names[i]}: kP={self.kP_turn[i]:.6f} kI={self.kI_turn[i]:.8f} kD={self.kD_turn[i]:.6f}")
			else:
				# Old flat format - only has drive gains
				print("[DEBUG] Using legacy flat format (drive gains only)")
				self.kP = [
					gains_data["FL"].get("kP", self.kP[0]),
					gains_data["FR"].get("kP", self.kP[1]),
					gains_data["RL"].get("kP", self.kP[2]),
					gains_data["RR"].get("kP", self.kP[3]),
				]
				self.kI = [
					gains_data["FL"].get("kI", self.kI[0]),
					gains_data["FR"].get("kI", self.kI[1]),
					gains_data["RL"].get("kI", self.kI[2]),
					gains_data["RR"].get("kI", self.kI[3]),
				]
				self.kD = [
					gains_data["FL"].get("kD", self.kD[0]),
					gains_data["FR"].get("kD", self.kD[1]),
					gains_data["RL"].get("kD", self.kD[2]),
					gains_data["RR"].get("kD", self.kD[3]),
				]
				
				print(f"[LOADED] PID gains loaded from {filepath} (legacy format)")
				for i in range(4):
					print(f"[LOADED] {names[i]}: kP={self.kP[i]:.6f} kI={self.kI[i]:.8f} kD={self.kD[i]:.6f}")
			
			return True
		except Exception as e:
			print(f"[ERROR] Failed to load PID gains: {e}")
			return False
	
	def get_encoder_count_for_motor(self, index: int) -> int:
		"""Get encoder count by motor index."""
		if index == 0:
			return self.front_left.get_encoder_count()
		elif index == 1:
			return self.front_right.get_encoder_count()
		elif index == 2:
			return self.rear_left.get_encoder_count()
		else:
			return self.rear_right.get_encoder_count()


