# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile

try:
	from rev import SparkMaxConfig, SparkLowLevel, AbsoluteEncoderConfig
	HAS_REV = True
except (ImportError, ModuleNotFoundError):
	HAS_REV = False
	SparkMaxConfig = None
	SparkLowLevel = None
	AbsoluteEncoderConfig = None


class DriveConstants:
	"""Drive constants for the robot."""
	
	# Driving Parameters - Note that these are not the maximum capable speeds of
	# the robot, rather the allowed maximum speeds
	k_max_speed_meters_per_second = 4.8
	k_max_angular_speed = 2 * math.pi  # radians per second

	# Chassis configuration
	k_track_width = 27.0 * 0.0254  # 27 inches in meters
	# Distance between centers of right and left wheels on robot
	k_wheel_base = 27.0 * 0.0254  # 27 inches in meters
	# Distance between front and back wheels on robot
	k_drive_kinematics = SwerveDrive4Kinematics(
		Translation2d(k_wheel_base / 2, k_track_width / 2),
		Translation2d(k_wheel_base / 2, -k_track_width / 2),
		Translation2d(-k_wheel_base / 2, k_track_width / 2),
		Translation2d(-k_wheel_base / 2, -k_track_width / 2))

	# Angular offsets of the modules relative to the chassis in radians
	_k_easy_swerve_angular_offset_compensation = math.pi / 4
	k_front_left_chassis_angular_offset = (-math.pi / 2) + _k_easy_swerve_angular_offset_compensation
	k_front_right_chassis_angular_offset = 0 + _k_easy_swerve_angular_offset_compensation
	k_rear_left_chassis_angular_offset = math.pi + _k_easy_swerve_angular_offset_compensation
	k_rear_right_chassis_angular_offset = (math.pi / 2) + _k_easy_swerve_angular_offset_compensation

	# Encoder mechanical offsets (calibration hole position) - one per wheel
	# These values are the raw encoder readings when the wheel is at zero position
	# Measure by checking the raw encoder value when the wrench calibration hole is at 0°
	k_front_left_encoder_offset = 0.0   # Calibrated from latest output
	k_front_right_encoder_offset = 0.0    # Right side reference (0 radians)
	k_rear_left_encoder_offset = 0.01457281748491359    # Calibrated from latest output
	k_rear_right_encoder_offset = 0.0     # Right side reference (0 radians)

	# The EasySwerve module allows installation of the motors either on top or bottom of the module.
	# These constants configure the location of the motors. The default configuration is with both
	# motors on the bottom of the module.
	k_front_left_driving_motor_on_bottom = True
	k_rear_left_driving_motor_on_bottom = True
	k_front_right_driving_motor_on_bottom = True
	k_rear_right_driving_motor_on_bottom = True

	k_front_left_turning_motor_on_bottom = True
	k_rear_left_turning_motor_on_bottom = True
	k_front_right_turning_motor_on_bottom = True
	k_rear_right_turning_motor_on_bottom = True

	# SPARK MAX CAN IDs
	k_front_right_driving_can_id = 2
	k_front_right_turning_can_id = 9

	k_rear_right_turning_can_id = 3
	k_rear_right_driving_can_id = 4

	k_rear_left_turning_can_id = 5
	k_rear_left_driving_can_id = 6

	
	k_front_left_driving_can_id = 8
	k_front_left_turning_can_id = 7

	k_gyro_reversed = False


class NeoMotorConstants:
	"""NEO motor constants."""
	
	k_free_speed_rpm = 5676


class ModuleConstants:
	"""Module constants for swerve modules."""
	
	# The EasySwerve module can only be configured with one pinion gears: 12T.
	k_driving_motor_pinion_teeth = 12

	# Calculations required for driving motor conversion factors and feed forward
	k_driving_motor_free_speed_rps = NeoMotorConstants.k_free_speed_rpm / 60
	k_wheel_diameter_meters = 4 * 0.0254  # 4 inches in meters
	k_wheel_circumference_meters = k_wheel_diameter_meters * math.pi
	# 45 teeth on the wheel's bevel gear, 30 teeth on the first-stage spur gear,
	# 15 teeth on the bevel pinion
	k_driving_wheel_bevel_gear_teeth = 45.0
	k_driving_wheel_first_stage_spur_gear_teeth = 30.0
	k_driving_motor_bevel_pinion_teeth = 15.0
	k_driving_motor_reduction = (k_driving_wheel_bevel_gear_teeth * k_driving_wheel_first_stage_spur_gear_teeth) / (k_driving_motor_pinion_teeth * k_driving_motor_bevel_pinion_teeth)
	k_drive_wheel_free_speed_rps = (k_driving_motor_free_speed_rps * k_wheel_circumference_meters) / k_driving_motor_reduction


class OIConstants:
	"""Operator Interface constants."""
	
	k_driver_controller_port = 0
	k_drive_deadband = 0.1


class TurnMotorGains:
	"""PID gains for turn motors (calculated via Ziegler-Nichols autotune)."""
	
	# These values are populated by autotune
	# To use them, run autotune with START+BACK, copy the printed values here:
	kp = 0.1000000000   # REPLACE WITH AUTOTUNED VALUE
	ki = 0.0000000000   # REPLACE WITH AUTOTUNED VALUE
	kd = 0.0000000000   # REPLACE WITH AUTOTUNED VALUE


class AutoConstants:
	"""Autonomous command constants."""
	
	k_max_speed_meters_per_second = 3
	k_max_acceleration_meters_per_second_squared = 3
	k_max_angular_speed_radians_per_second = math.pi
	k_max_angular_speed_radians_per_second_squared = math.pi

	k_p_x_controller = 1
	k_p_y_controller = 1
	k_p_theta_controller = 1

	# Constraint for the motion profiled robot angle controller
	k_theta_controller_constraints = TrapezoidProfile.Constraints(
		k_max_angular_speed_radians_per_second,
		k_max_angular_speed_radians_per_second_squared)


class DIORearLeftEncoderConstants:
	"""DIO-based absolute encoder constants for rear left turn motor."""
	
	# Rear left turn motor (CANID 5) - bore encoder connected to RoboRIO DIO
	# White wire to DIO0 (Channel A/Bit 0)
	# Yellow wire to DIO1 (Channel B/Bit 1)
	# Blue wire to DIO2 (Index/Z or Bit 2)
	k_rear_left_dio_channel_a = 1  # SWAPPED - was 0
	k_rear_left_dio_channel_b = 0  # SWAPPED - was 1
	k_rear_left_dio_index = 2


class DIORrontLeftEncoderConstants:
	"""DIO-based absolute encoder constants for front left turn motor."""
	
	# Front left turn motor (CANID 7) - bore encoder connected to RoboRIO DIO
	# White wire to DIO3 (Channel A)
	# Yellow wire to DIO4 (Channel B)
	# Blue wire to DIO5 (Index/Z)
	k_front_left_dio_channel_a = 4  # SWAPPED - was 3
	k_front_left_dio_channel_b = 3  # SWAPPED - was 4
	k_front_left_dio_index = 5


class EasySwerveModuleConfig:
	"""Configuration for EasySwerve modules."""
	
	def __init__(self):
		"""Initialize driving and turning configurations."""
		if not HAS_REV:
			self.driving_config = None
			self.turning_config = None
			return
		
		# Use module constants to calculate conversion factors and feed forward gain.
		driving_factor = ModuleConstants.k_wheel_diameter_meters * 3.141592653589793 / ModuleConstants.k_driving_motor_reduction
		turning_factor = 2 * 3.141592653589793

		nominal_voltage = 12.0
		driving_velocity_feed_forward = nominal_voltage / ModuleConstants.k_drive_wheel_free_speed_rps

		# Driving configuration
		self.driving_config = SparkMaxConfig()
		self.driving_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
		self.driving_config.smartCurrentLimit(60)  # 70 for SparkFlex
		self.driving_config.encoder.positionConversionFactor(driving_factor)  # meters
		self.driving_config.encoder.velocityConversionFactor(driving_factor / 60.0)  # meters per second
		self.driving_config.closedLoop.P(0.04).I(0).D(0)
		self.driving_config.closedLoop.outputRange(-1, 1)

		# Turning configuration - Using ABSOLUTE encoder (Through Bore Encoder V2)
		self.turning_config = SparkMaxConfig()
		self.turning_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
		self.turning_config.smartCurrentLimit(40)  # 70 for SparkFlex
		self.turning_config.absoluteEncoder.inverted(False)
		self.turning_config.absoluteEncoder.positionConversionFactor(turning_factor)  # radians
		self.turning_config.absoluteEncoder.velocityConversionFactor(turning_factor / 60.0)  # radians per second
		self.turning_config.absoluteEncoder.apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2())
		self.turning_config.closedLoop.P(1).I(0).D(0)
		self.turning_config.closedLoop.outputRange(-1, 1)


# Global configuration instance (only created if rev is available)
easy_swerve_module_config = EasySwerveModuleConfig() if HAS_REV else None

