# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDriveKinematics
from wpimath.trajectory import TrapezoidProfile
from rev import SparkMaxConfig, SparkLowLevel, AbsoluteEncoderConfig


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
	k_drive_kinematics = SwerveDriveKinematics(
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
	k_front_left_driving_can_id = 9
	k_rear_left_driving_can_id = 15
	k_front_right_driving_can_id = 11
	k_rear_right_driving_can_id = 13

	k_front_left_turning_can_id = 8
	k_rear_left_turning_can_id = 14
	k_front_right_turning_can_id = 10
	k_rear_right_turning_can_id = 12

	k_gyro_reversed = False


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


class NeoMotorConstants:
	"""NEO motor constants."""
	
	k_free_speed_rpm = 5676


class EasySwerveModuleConfig:
	"""Configuration for EasySwerve modules."""
	
	def __init__(self):
		"""Initialize driving and turning configurations."""
		# Use module constants to calculate conversion factors and feed forward gain.
		driving_factor = ModuleConstants.k_wheel_diameter_meters * 3.141592653589793 / ModuleConstants.k_driving_motor_reduction
		turning_factor = 2 * 3.141592653589793

		nominal_voltage = 12.0
		driving_velocity_feed_forward = nominal_voltage / ModuleConstants.k_drive_wheel_free_speed_rps

		# Driving configuration
		self.driving_config = SparkMaxConfig()
		self.driving_config.set_idle_mode(SparkLowLevel.IdleMode.kBrake)
		self.driving_config.smart_current_limit(50)  # 70 for SparkFlex
		self.driving_config.encoder.position_conversion_factor(driving_factor)  # meters
		self.driving_config.encoder.velocity_conversion_factor(driving_factor / 60.0)  # meters per second
		self.driving_config.closed_loop.feedback_sensor(SparkMaxConfig.FeedbackSensor.kPrimaryEncoder)
		self.driving_config.closed_loop.p(0.04).i(0).d(0)
		self.driving_config.closed_loop.output_range(-1, 1)
		self.driving_config.closed_loop.feed_forward.kV(driving_velocity_feed_forward)

		# Turning configuration
		self.turning_config = SparkMaxConfig()
		self.turning_config.set_idle_mode(SparkLowLevel.IdleMode.kBrake)
		self.turning_config.smart_current_limit(20)  # 70 for SparkFlex
		self.turning_config.absolute_encoder.inverted(False)
		self.turning_config.absolute_encoder.position_conversion_factor(turning_factor)  # radians
		self.turning_config.absolute_encoder.velocity_conversion_factor(turning_factor / 60.0)  # radians per second
		self.turning_config.absolute_encoder.apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2)

		self.turning_config.closed_loop.feedback_sensor(SparkMaxConfig.FeedbackSensor.kAbsoluteEncoder)
		self.turning_config.closed_loop.p(1).i(0).d(0)
		self.turning_config.closed_loop.output_range(-1, 1)
		# Enable PID wrap around for the turning motor. This will allow the PID
		# controller to go through 0 to get to the setpoint i.e. going from 350 degrees
		# to 10 degrees will go through 0 rather than the other direction which is a
		# longer route.
		self.turning_config.closed_loop.position_wrapping_enabled(True)
		self.turning_config.closed_loop.position_wrapping_input_range(0, turning_factor)


# Global configuration instance
easy_swerve_module_config = EasySwerveModuleConfig()
