# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

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
			driving_motor_on_bottom: bool, turning_motor_on_bottom: bool, module_name: str = "Module"):
		"""
		Initialize an EasySwerve module.

		Args:
			driving_can_id: CAN ID of the driving motor
			turning_can_id: CAN ID of the turning motor
			chassis_angular_offset: Angular offset of the module
			driving_motor_on_bottom: Whether driving motor is on the bottom
			turning_motor_on_bottom: Whether turning motor is on the bottom
			module_name: Name for debugging (e.g., "Front Left")
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
		
		if not HAS_REV:
			print(f"[{module_name}] REV not available")
			return

		self.driving_spark = SparkMax(driving_can_id, SparkLowLevel.MotorType.kBrushless)
		self.turning_spark = SparkMax(turning_can_id, SparkLowLevel.MotorType.kBrushless)
		print(f"[{module_name}] Motors initialized - Drive CAN {driving_can_id}, Turn CAN {turning_can_id}")

		self.driving_encoder = self.driving_spark.getEncoder()
		self.turning_encoder = self.turning_spark.getAbsoluteEncoder()
		
		if self.turning_encoder is None:
			print(f"[{module_name}] WARNING: Turning encoder is None!")

		self.driving_closed_loop_controller = self.driving_spark.getClosedLoopController()
		self.turning_closed_loop_controller = self.turning_spark.getClosedLoopController()

		print(f"[{module_name}] Using hardware client configuration only - no code-level config applied")

		self.desired_state = SwerveModuleState(0.0, Rotation2d(self.turning_encoder.getPosition()))
		self.driving_encoder.setPosition(0)

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
		return SwerveModuleState(
			self.driving_encoder.getVelocity(),
			Rotation2d(self.turning_encoder.getPosition() - self.chassis_angular_offset))

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
		return SwerveModulePosition(
			self.driving_encoder.getPosition(),
			Rotation2d(self.turning_encoder.getPosition() - self.chassis_angular_offset))

	def set_desired_state(self, desired_state: SwerveModuleState):
		"""
		Sets the desired state for the module.

		Args:
			desired_state: Desired state with speed and angle.
		"""
		if not HAS_REV or self.turning_spark is None:
			return
		
		# Apply chassis angular offset to the desired state.
		corrected_desired_state = SwerveModuleState()
		corrected_desired_state.speed = desired_state.speed
		corrected_desired_state.angle = desired_state.angle.rotateBy(Rotation2d(self.chassis_angular_offset))

		# Optimize the reference state to avoid spinning further than 90 degrees.
		current_angle = self.turning_encoder.getPosition()
		corrected_desired_state.optimize(Rotation2d(current_angle))

		# Command driving SPARK towards its setpoint
		self.driving_closed_loop_controller.setReference(
			corrected_desired_state.speed, SparkLowLevel.ControlType.kVelocity)
		
		# For turning: Use open-loop PWM with simple P control
		# Closed-loop doesn't work well on all motors (likely CAN saturation)
		target_angle = corrected_desired_state.angle.radians()
		angle_error = target_angle - current_angle
		
		# Normalize to -pi to pi
		while angle_error > 3.14159:
			angle_error -= 6.28318
		while angle_error < -3.14159:
			angle_error += 6.28318
		
		# P controller with gain 1.0
		pwm_output = max(-1.0, min(1.0, angle_error * 1.0))
		self.turning_spark.set(pwm_output)
		self.last_pwm = pwm_output

		# DEBUG
		applied_output = self.turning_spark.getAppliedOutput()
		self.loop_counter += 1
		if self.loop_counter % 250 == 0:
			print(f"[{self.module_name}] pos={current_angle:.3f} tgt={target_angle:.3f} err={angle_error:.3f} cmd={pwm_output:.3f} out={applied_output:.3f}")
			if abs(pwm_output) > 0.1:
				print(f"  ^^ {self.module_name} IS COMMANDING MOVEMENT ^^")

		self.desired_state = desired_state

	def reset_encoders(self):
		"""Zeroes all the SwerveModule encoders."""
		if not HAS_REV or self.driving_encoder is None:
			return
		self.driving_encoder.setPosition(0)

	def print_diagnostics(self):
		"""Print motor diagnostics and settings."""
		if not HAS_REV or self.turning_spark is None:
			return
		
		try:
			print(f"\n[{self.module_name}] === DIAGNOSTICS ===")
			print(f"  Turning Motor CAN ID: {self.turning_spark.getDeviceId()}")
			print(f"  Temp: {self.turning_spark.getMotorTemperature():.1f}C")
			print(f"  Bus Voltage: {self.turning_spark.getBusVoltage():.1f}V")
			print(f"  Applied Output: {self.turning_spark.getAppliedOutput():.3f}")
			print(f"  Output Current: {self.turning_spark.getOutputCurrent():.2f}A")
			print(f"  Encoder Position: {self.turning_encoder.getPosition():.3f} rad")
			print(f"  Encoder Velocity: {self.turning_encoder.getVelocity():.3f} rad/s")
			
			# Try to read status
			try:
				status = self.turning_spark.getLastError()
				print(f"  Last Error: {status}")
			except:
				pass
			
		except Exception as e:
			print(f"[{self.module_name}] Diagnostic error: {e}")
