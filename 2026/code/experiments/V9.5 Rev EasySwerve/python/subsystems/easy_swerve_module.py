# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import math
try:
	from rev import SparkMax, SparkLowLevel, PersistMode, ResetMode, AbsoluteEncoder, RelativeEncoder, SparkClosedLoopController, SparkMaxConfig
	HAS_REV = True
except (ImportError, ModuleNotFoundError):
	HAS_REV = False
	SparkMax = None
	SparkLowLevel = None
	PersistMode = None
	ResetMode = None
	AbsoluteEncoder = None
	RelativeEncoder = None
	SparkClosedLoopController = None
	SparkMaxConfig = None

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from constants import easy_swerve_module_config


class EasySwerveModule:
	"""
	Represents an EasySwerve swerve module for the robot.

	Constructs an EasySwerveModule and configures the driving and turning motor,
	encoder, and PID controller. This configuration is specific to the REV
	EasySwerve Module built with NEOs, SPARK MAXs, and a Through Bore
	Encoder V2.
	"""

	def __init__(self, driving_can_id: int, turning_can_id: int, chassis_angular_offset: float,
			driving_motor_on_bottom: bool, turning_motor_on_bottom: bool, module_name: str = "Module", encoder_offset: float = None):
		"""
		Initialize an EasySwerve module.

		Args:
			driving_can_id: CAN ID of the driving motor
			turning_can_id: CAN ID of the turning motor
			chassis_angular_offset: Angular offset of the module
			driving_motor_on_bottom: Whether driving motor is on the bottom
			turning_motor_on_bottom: Whether turning motor is on the bottom
			module_name: Name for debugging (e.g., "Front Left")
			encoder_offset: Raw encoder value at zero wheel position (calibration)
		"""
		self.module_name = module_name
		self.driving_spark = None
		self.turning_spark = None
		self.driving_encoder = None
		self.turning_encoder = None
		self.driving_closed_loop_controller = None
		self.turning_closed_loop_controller = None
		self.chassis_angular_offset = chassis_angular_offset
		self.desired_state = None
		self.loop_counter = 0
		self.last_pwm = 0  # Track last PWM for debugging
		self.last_target_angle = 0.0
		self.last_command_pwm = 0.0
		self.command_count = 0
		self.skipped_updates = 0
		self.debug_enabled = False  # Only print when enabled
		self.integral_accumulator = 0.0  # For I term in PI control
		
		# Encoder mechanical offset - use provided value or default to π/4
		if encoder_offset is not None:
			self.encoder_mechanical_offset = encoder_offset
		else:
			self.encoder_mechanical_offset = math.pi / 4  # Default: π/4 radians = 45°
		
		# Autotune state variables
		self.autotune_active = False
		self.autotune_start_time = None
		self.autotune_peaks = []
		self.autotune_last_position = 0.0
		self.autotune_last_direction = 0.0
		
		# PID gains (autotuned or manual)
		self.turn_kp = 0.1  # Default P gain (will be overwritten by autotune)
		self.turn_ki = 0.0  # Default I gain
		self.turn_kd = 0.0  # Default D gain
		
		self.gains_initialized = False  # Track if we've printed the startup message
		
		# Debug output tracking - only print when values change
		self.last_printed_angle = None
		self.last_printed_error = None
		self.last_printed_pwm = None
		self.startup_printed = False  # Print once on first cycle
		
		if not HAS_REV:
			print(f"[{module_name}] REV not available")
			return

		self.driving_spark = SparkMax(driving_can_id, SparkLowLevel.MotorType.kBrushless)
		self.turning_spark = SparkMax(turning_can_id, SparkLowLevel.MotorType.kBrushless)
		self.driving_encoder = self.driving_spark.getEncoder()
		self.turning_encoder = self.turning_spark.getAbsoluteEncoder()
		
		if self.turning_encoder is None:
			print(f"[{self.module_name}] WARNING: Turning encoder is None!")

		self.driving_closed_loop_controller = self.driving_spark.getClosedLoopController()
		self.turning_closed_loop_controller = self.turning_spark.getClosedLoopController()

		self.desired_state = SwerveModuleState(0.0, Rotation2d(self.turning_encoder.getPosition()))
		self.driving_encoder.setPosition(0)
		
		# Load autotuned gains from constants if available
		from constants import TurnMotorGains
		if hasattr(TurnMotorGains, 'kp') and TurnMotorGains.kp != 0.0:
			self.turn_kp = TurnMotorGains.kp
			self.turn_ki = TurnMotorGains.ki
			self.turn_kd = TurnMotorGains.kd
			self.gains_initialized = True
			print(f"[{module_name}] OK - Loaded autotuned gains from constants.py")
		else:
			# Gains are zero - print prompt once
			if not self.gains_initialized:
				print(f"[{module_name}] WARNING - Turn motor gains not tuned. Press START+BACK during teleop to autotune.")
				self.gains_initialized = True

	def get_state(self) -> SwerveModuleState:
		"""
		Returns the current state of the module.

		Returns:
			The current state of the module.
		"""
		if not HAS_REV or self.driving_encoder is None:
			return SwerveModuleState(0.0, Rotation2d(0.0))
		# Apply chassis angular offset to the encoder position to get the position
		# relative to the chassis.
		raw_turning_position = self.turning_encoder.getPosition()
		# Normalize to -pi to pi
		normalized_angle = raw_turning_position
		while normalized_angle > 3.14159:
			normalized_angle -= 6.28318
		while normalized_angle < -3.14159:
			normalized_angle += 6.28318
		
		return SwerveModuleState(
			self.driving_encoder.getVelocity(),
			Rotation2d(normalized_angle - self.chassis_angular_offset))

	def periodic_turn_command(self):
		"""
		Called EVERY cycle to command the turning motor using closed-loop control.
		Uses SPARK MAX built-in PID with Through-Bore Encoder V2 from Hardware Client config.
		"""
		if not HAS_REV or self.turning_spark is None or self.turning_closed_loop_controller is None:
			return
		
		# Update autotune if active
		if self.autotune_active:
			kp, ki, kd = self.update_turn_autotune()
			if kp is not None:
				# Autotune complete
				print(f"[{self.module_name}] Autotune complete - gains applied")
				print(f"[{self.module_name}] kP={kp:.8f}, kI={ki:.10f}")
			return  # Don't run normal control loop during autotune
		
		# Get current position from encoder (continuous multi-turn value)
		raw_encoder_position = self.turning_encoder.getPosition()
		
		# Normalize raw encoder to single rotation (-π to π)
		raw_normalized = raw_encoder_position
		while raw_normalized > 3.14159:
			raw_normalized -= 6.28318
		while raw_normalized < -3.14159:
			raw_normalized += 6.28318
		
		# Apply mechanical offset (calibration hole position offset)
		current_angle = raw_normalized - self.encoder_mechanical_offset
		
		# Keep current_angle in -π to π range
		while current_angle > 3.14159:
			current_angle -= 6.28318
		while current_angle < -3.14159:
			current_angle += 6.28318
		
		# Send target to SPARK MAX closed-loop controller
		# SPARK MAX firmware handles PID using Hardware Client configuration
		self.turning_closed_loop_controller.setReference(
			self.last_target_angle,
			SparkLowLevel.ControlType.kPosition)
		
		# Get applied output for debugging
		applied_output = self.turning_spark.getAppliedOutput()
		
		# Print startup status once - shows raw encoder for calibration
		if not self.startup_printed:
			angle_error = self.last_target_angle - current_angle
			while angle_error > 3.14159:
				angle_error -= 6.28318
			while angle_error < -3.14159:
				angle_error += 6.28318
			print(f"[{self.module_name}] START: raw_encoder={raw_encoder_position:7.3f} offset={self.encoder_mechanical_offset:7.3f} adjusted={current_angle:7.3f} tgt={self.last_target_angle:7.3f} applied={applied_output:6.3f}")
			self.startup_printed = True
		
		# Only print when position changes significantly
		self.loop_counter += 1
		if self.last_printed_angle is None or abs(current_angle - self.last_printed_angle) > 0.05:
			self.last_printed_angle = current_angle
			angle_error = self.last_target_angle - current_angle
			while angle_error > 3.14159:
				angle_error -= 6.28318
			while angle_error < -3.14159:
				angle_error += 6.28318
			print(f"[{self.module_name}] raw={raw_encoder_position:7.3f} adj={current_angle:7.3f} err={angle_error:7.3f} applied={applied_output:6.3f}")

	def get_position(self) -> SwerveModulePosition:
		"""
		Returns the current position of the module.

		Returns:
			The current position of the module.
		"""
		if not HAS_REV or self.driving_encoder is None:
			return SwerveModulePosition(0.0, Rotation2d(0.0))
		# Apply chassis angular offset to the encoder position to get the position
		# relative to the chassis.
		raw_turning_position = self.turning_encoder.getPosition()
		# Normalize to -pi to pi
		normalized_angle = raw_turning_position
		while normalized_angle > 3.14159:
			normalized_angle -= 6.28318
		while normalized_angle < -3.14159:
			normalized_angle += 6.28318
		
		return SwerveModulePosition(
			self.driving_encoder.getPosition(),
			Rotation2d(normalized_angle - self.chassis_angular_offset))

	def set_desired_state(self, desired_state: SwerveModuleState):
		"""
		Sets the desired state for the module.
		Only updates the target - actual motor command happens in periodic_turn_command().

		Args:
			desired_state: Desired state with speed and angle.
		"""
		if not HAS_REV or self.turning_spark is None:
			return
		
		# NOTE: NOT applying chassis angular offset here - it should only affect feedback/odometry
		# The kinematics system already accounts for module positions when calculating desired states
		corrected_desired_state = SwerveModuleState()
		corrected_desired_state.speed = desired_state.speed
		corrected_desired_state.angle = desired_state.angle

		# Optimize the reference state to avoid spinning further than 90 degrees.
		# Pass RAW encoder position (0 to 2π) to optimize()
		raw_encoder_position = self.turning_encoder.getPosition()
		corrected_desired_state.optimize(Rotation2d(raw_encoder_position))

		# Command driving SPARK towards its setpoint
		self.driving_closed_loop_controller.setReference(
			corrected_desired_state.speed, SparkLowLevel.ControlType.kVelocity)
		
		# UPDATE TARGET - normalize to -pi to pi for motor control
		self.last_target_angle = corrected_desired_state.angle.radians()
		# Normalize target to -pi to pi
		while self.last_target_angle > 3.14159:
			self.last_target_angle -= 6.28318
		while self.last_target_angle < -3.14159:
			self.last_target_angle += 6.28318
		
		# Reset integral accumulator when target changes
		self.integral_accumulator = 0.0
		
		self.command_count += 1

		# DEBUG
		# Get normalized encoder for display
		norm_encoder = raw_encoder_position
		while norm_encoder > 3.14159:
			norm_encoder -= 6.28318
		while norm_encoder < -3.14159:
			norm_encoder += 6.28318
			
		angle_error = self.last_target_angle - norm_encoder
		while angle_error > 3.14159:
			angle_error -= 6.28318
		while angle_error < -3.14159:
			angle_error += 6.28318
		
		# Only print UPDATE during debug mode (not by default)
		if self.debug_enabled:
			print(f"[{self.module_name}] UPDATE#{self.command_count:04d} raw={raw_encoder_position:8.3f} norm={norm_encoder:7.3f} desired={desired_state.angle.radians():7.3f} tgt={self.last_target_angle:7.3f} err={angle_error:7.3f}")

		self.desired_state = desired_state

	def start_turn_autotune(self):
		"""Start Ziegler-Nichols relay autotune for turn motor."""
		import time
		print(f"[{self.module_name}] === STARTING TURN MOTOR AUTOTUNE (8 seconds) ===")
		self.autotune_active = True
		self.autotune_start_time = time.time()
		self.autotune_relay_direction = 1  # +0.3 PWM
		self.autotune_last_switch = time.time()
		self.autotune_relay_power = 0.3
		self.autotune_switch_interval = 0.5  # Switch every 500ms
		self.autotune_peaks = []  # Store peak positions
		self.autotune_last_position = 0.0
		self.autotune_direction_changed = False

	def update_turn_autotune(self):
		"""Call every cycle to run autotune. Returns (kP, kI, kD) when complete, or (None, None, None) if running."""
		if not self.autotune_active:
			return None, None, None
		
		import time
		
		# Apply relay command
		relay_cmd = self.autotune_relay_direction * self.autotune_relay_power
		self.turning_spark.set(relay_cmd)
		
		# Get current position
		current_position = self.turning_encoder.getPosition()
		
		# Check if direction changes (oscillation detection)
		position_change = current_position - self.autotune_last_position
		
		# Detect direction change and record peak
		if len(self.autotune_peaks) == 0:
			self.autotune_last_direction = position_change
		else:
			if (self.autotune_last_direction > 0 and position_change < 0) or (self.autotune_last_direction < 0 and position_change > 0):
				# Direction changed - record peak
				self.autotune_peaks.append(abs(current_position))
				print(f"[{self.module_name}] PEAK #{len(self.autotune_peaks)}: position={current_position:.4f}")
			self.autotune_last_direction = position_change
		
		self.autotune_last_position = current_position
		
		# Check if time to switch relay direction
		if time.time() - self.autotune_last_switch > self.autotune_switch_interval:
			self.autotune_relay_direction *= -1
			self.autotune_last_switch = time.time()
			print(f"[{self.module_name}] RELAY SWITCHED to {self.autotune_relay_direction:+.1f}")
		
		# Check if autotune duration complete
		if time.time() - self.autotune_start_time > 8.0:
			self.autotune_active = False
			self.turning_spark.set(0)  # Stop motor
			
			# Calculate gains from peaks
			return self._finalize_autotune()
		
		return None, None, None

	def _finalize_autotune(self):
		"""Calculate PID gains from relay response using Ziegler-Nichols method."""
		if len(self.autotune_peaks) < 2:
			print(f"[{self.module_name}] ERROR: Not enough peaks ({len(self.autotune_peaks)}) to calculate gains")
			return None, None, None
		
		# Calculate amplitude from peak-to-peak
		amplitude = (self.autotune_peaks[-1] - self.autotune_peaks[0]) / len(self.autotune_peaks)
		
		# Period: 2 switches per full oscillation, each switch is switch_interval
		period = 2.0 * self.autotune_switch_interval * 2  # 2 directional changes = 1 full period
		frequency = 1.0 / period if period > 0 else 1.0
		
		# Ultimate Gain (Ku) from relay amplitude and output
		ku = (4.0 * self.autotune_relay_power) / (math.pi * amplitude) if amplitude > 0 else 1.0
		pu = period
		
		# Ziegler-Nichols PI gains (no D term for stability)
		kp = 0.45 * ku
		ki = 0.54 * ku / pu if pu > 0 else 0.001
		kd = 0.0  # No D term
		
		print(f"\n[{self.module_name}] ===== AUTOTUNE RESULTS =====")
		print(f"[{self.module_name}] Peaks recorded: {len(self.autotune_peaks)}")
		print(f"[{self.module_name}] Amplitude: {amplitude:.6f} rad")
		print(f"[{self.module_name}] Period: {period:.4f} s, Frequency: {frequency:.4f} Hz")
		print(f"[{self.module_name}] Ultimate Gain (Ku): {ku:.6f}")
		print(f"[{self.module_name}] Ultimate Period (Pu): {pu:.6f} s")
		print(f"[{self.module_name}] === CALCULATED GAINS ===")
		print(f"[{self.module_name}] kP = {kp:.8f}")
		print(f"[{self.module_name}] kI = {ki:.10f}")
		print(f"[{self.module_name}] kD = {kd:.8f}")
		print(f"[{self.module_name}] =============================\n")
		
		return kp, ki, kd

	def set_turn_gains(self, kp: float, ki: float = 0.0, kd: float = 0.0):
		"""Manually set turn motor PID gains."""
		self.turn_kp = kp
		self.turn_ki = ki
		self.turn_kd = kd
		self.integral_accumulator = 0.0
		print(f"[{self.module_name}] Gains set: kP={kp:.8f}, kI={ki:.10f}, kD={kd:.8f}")

	def reset_encoders(self):
		"""Zeroes all the SwerveModule encoders."""
		if not HAS_REV or self.driving_encoder is None:
			return
		self.driving_encoder.setPosition(0)

	def print_sparkmax_config(self):
		"""Print all SparkMax configuration values WITHOUT writing to firmware."""
		if not HAS_REV:
			print(f"[{self.module_name}] REV API not available")
			return
		
		print(f"\n{'='*80}")
		print(f"[{self.module_name}] === SPARKMAX CONFIGURATION DUMP ===")
		print(f"{'='*80}")
		
		# DRIVING MOTOR
		if self.driving_spark:
			print(f"\n[{self.module_name}] DRIVING MOTOR (CAN ID: {self.driving_spark.getDeviceId()})")
			print(f"  Applied Output: {self.driving_spark.getAppliedOutput():.3f}")
			print(f"  Bus Voltage: {self.driving_spark.getBusVoltage():.2f}V")
			print(f"  Output Current: {self.driving_spark.getOutputCurrent():.2f}A")
			print(f"  Motor Temp: {self.driving_spark.getMotorTemperature():.1f}°C")
			
			if self.driving_encoder:
				print(f"  Encoder Position: {self.driving_encoder.getPosition():.6f} m")
				print(f"  Encoder Velocity: {self.driving_encoder.getVelocity():.6f} m/s")
				print(f"  Encoder PosFactor: {self.driving_encoder.getPositionConversionFactor():.6f}")
				print(f"  Encoder VelFactor: {self.driving_encoder.getVelocityConversionFactor():.6f}")
			
			if self.driving_closed_loop_controller:
				print(f"  Closed Loop Reference: {self.driving_closed_loop_controller.getReference():.6f}")
		else:
			print(f"  [WARNING] Driving motor not initialized!")
		
		# TURNING MOTOR
		if self.turning_spark:
			print(f"\n[{self.module_name}] TURNING MOTOR (CAN ID: {self.turning_spark.getDeviceId()})")
			print(f"  Applied Output: {self.turning_spark.getAppliedOutput():.3f}")
			print(f"  Bus Voltage: {self.turning_spark.getBusVoltage():.2f}V")
			print(f"  Output Current: {self.turning_spark.getOutputCurrent():.2f}A")
			print(f"  Motor Temp: {self.turning_spark.getMotorTemperature():.1f}°C")
			
			if self.turning_encoder:
				print(f"  Encoder Position (raw): {self.turning_encoder.getPosition():.6f} rad")
				print(f"  Encoder Velocity: {self.turning_encoder.getVelocity():.6f} rad/s")
				print(f"  Encoder PosFactor: {self.turning_encoder.getPositionConversionFactor():.6f}")
				print(f"  Encoder VelFactor: {self.turning_encoder.getVelocityConversionFactor():.6f}")
				print(f"  Mechanical Offset: {self.encoder_mechanical_offset:.6f} rad")
			
			if self.turning_closed_loop_controller:
				print(f"  Closed Loop Reference: {self.turning_closed_loop_controller.getReference():.6f}")
			
			print(f"  PID Gains: kP={self.turn_kp:.8f}, kI={self.turn_ki:.10f}, kD={self.turn_kd:.8f}")
		else:
			print(f"  [WARNING] Turning motor not initialized!")
		
		print(f"{'='*80}\n")
