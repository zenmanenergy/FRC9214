# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from rev import SparkMax, SparkLowLevel, PersistMode, ResetMode, AbsoluteEncoder, RelativeEncoder, SparkClosedLoopController
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
			driving_motor_on_bottom: bool, turning_motor_on_bottom: bool):
		"""
		Initialize an EasySwerve module.

		Args:
			driving_can_id: CAN ID of the driving motor
			turning_can_id: CAN ID of the turning motor
			chassis_angular_offset: Angular offset of the module
			driving_motor_on_bottom: Whether driving motor is on the bottom
			turning_motor_on_bottom: Whether turning motor is on the bottom
		"""
		self.driving_spark = SparkMax(driving_can_id, SparkLowLevel.MotorType.kBrushless)
		self.turning_spark = SparkMax(turning_can_id, SparkLowLevel.MotorType.kBrushless)

		self.driving_encoder: RelativeEncoder = self.driving_spark.get_encoder()
		self.turning_encoder: AbsoluteEncoder = self.turning_spark.get_absolute_encoder()

		self.driving_closed_loop_controller: SparkClosedLoopController = self.driving_spark.get_closed_loop_controller()
		self.turning_closed_loop_controller: SparkClosedLoopController = self.turning_spark.get_closed_loop_controller()

		# Apply the respective configurations to the SPARKs. Reset parameters before
		# applying the configuration to bring the SPARK to a known good state. Persist
		# the settings to the SPARK to avoid losing them on a power cycle.
		driving_config = easy_swerve_module_config.driving_config
		driving_config.inverted(driving_motor_on_bottom)
		self.driving_spark.configure(driving_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

		turning_config = easy_swerve_module_config.turning_config
		turning_config.inverted(not turning_motor_on_bottom)
		self.turning_spark.configure(turning_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

		self.chassis_angular_offset = chassis_angular_offset
		self.desired_state = SwerveModuleState(0.0, Rotation2d(self.turning_encoder.get_position()))
		self.driving_encoder.set_position(0)

	def get_state(self) -> SwerveModuleState:
		"""
		Returns the current state of the module.

		Returns:
			The current state of the module.
		"""
		# Apply chassis angular offset to the encoder position to get the position
		# relative to the chassis.
		return SwerveModuleState(
			self.driving_encoder.get_velocity(),
			Rotation2d(self.turning_encoder.get_position() - self.chassis_angular_offset))

	def get_position(self) -> SwerveModulePosition:
		"""
		Returns the current position of the module.

		Returns:
			The current position of the module.
		"""
		# Apply chassis angular offset to the encoder position to get the position
		# relative to the chassis.
		return SwerveModulePosition(
			self.driving_encoder.get_position(),
			Rotation2d(self.turning_encoder.get_position() - self.chassis_angular_offset))

	def set_desired_state(self, desired_state: SwerveModuleState):
		"""
		Sets the desired state for the module.

		Args:
			desired_state: Desired state with speed and angle.
		"""
		# Apply chassis angular offset to the desired state.
		corrected_desired_state = SwerveModuleState()
		corrected_desired_state.speed = desired_state.speed
		corrected_desired_state.angle = desired_state.angle.rotateBy(Rotation2d.from_radians(self.chassis_angular_offset))

		# Optimize the reference state to avoid spinning further than 90 degrees.
		corrected_desired_state.optimize(Rotation2d(self.turning_encoder.get_position()))

		# Command driving and turning SPARKs towards their respective setpoints.
		self.driving_closed_loop_controller.set_reference(
			corrected_desired_state.speed, SparkMaxConfig.ControlType.kVelocity)
		self.turning_closed_loop_controller.set_reference(
			corrected_desired_state.angle.radians(), SparkMaxConfig.ControlType.kPosition)

		self.desired_state = desired_state

	def reset_encoders(self):
		"""Zeroes all the SwerveModule encoders."""
		self.driving_encoder.set_position(0)
