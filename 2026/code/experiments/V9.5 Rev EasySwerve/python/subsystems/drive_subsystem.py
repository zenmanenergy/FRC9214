# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDriveKinematics, SwerveDriveOdometry, SwerveModulePosition, SwerveModuleState
from wpilib import ADIS16470_IMU
from wpilib.expression import Subsystem
from constants import DriveConstants
from subsystems.easy_swerve_module import EasySwerveModule


class DriveSubsystem(Subsystem):
	"""
	Represents the drivetrain subsystem with four swerve modules.
	"""

	def __init__(self):
		"""Creates a new DriveSubsystem."""
		super().__init__()
		
		# Create EasySwerveModules
		self.front_left = EasySwerveModule(
			DriveConstants.k_front_left_driving_can_id,
			DriveConstants.k_front_left_turning_can_id,
			DriveConstants.k_front_left_chassis_angular_offset,
			DriveConstants.k_front_left_driving_motor_on_bottom,
			DriveConstants.k_front_left_turning_motor_on_bottom)

		self.front_right = EasySwerveModule(
			DriveConstants.k_front_right_driving_can_id,
			DriveConstants.k_front_right_turning_can_id,
			DriveConstants.k_front_right_chassis_angular_offset,
			DriveConstants.k_front_right_driving_motor_on_bottom,
			DriveConstants.k_front_right_turning_motor_on_bottom)

		self.rear_left = EasySwerveModule(
			DriveConstants.k_rear_left_driving_can_id,
			DriveConstants.k_rear_left_turning_can_id,
			DriveConstants.k_rear_left_chassis_angular_offset,
			DriveConstants.k_rear_left_driving_motor_on_bottom,
			DriveConstants.k_rear_left_turning_motor_on_bottom)

		self.rear_right = EasySwerveModule(
			DriveConstants.k_rear_right_driving_can_id,
			DriveConstants.k_rear_right_turning_can_id,
			DriveConstants.k_rear_right_chassis_angular_offset,
			DriveConstants.k_rear_right_driving_motor_on_bottom,
			DriveConstants.k_rear_right_turning_motor_on_bottom)

		# The gyro sensor
		self.gyro = ADIS16470_IMU()

		# Odometry class for tracking robot pose
		self.odometry = SwerveDriveOdometry(
			DriveConstants.k_drive_kinematics,
			Rotation2d.from_degrees(self.gyro.get_angle()),
			[
				self.front_left.get_position(),
				self.front_right.get_position(),
				self.rear_left.get_position(),
				self.rear_right.get_position()
			],
			Pose2d())

	def periodic(self):
		"""Update the odometry in the periodic block."""
		self.odometry.update(
			Rotation2d.from_degrees(self.gyro.get_angle()),
			[
				self.front_left.get_position(),
				self.front_right.get_position(),
				self.rear_left.get_position(),
				self.rear_right.get_position()
			])

	def get_pose(self) -> Pose2d:
		"""
		Returns the currently-estimated pose of the robot.

		Returns:
			The pose.
		"""
		return self.odometry.get_pose()

	def reset_odometry(self, pose: Pose2d):
		"""
		Resets the odometry to the specified pose.

		Args:
			pose: The pose to which to set the odometry.
		"""
		self.odometry.reset_position(
			Rotation2d.from_degrees(self.gyro.get_angle()),
			[
				self.front_left.get_position(),
				self.front_right.get_position(),
				self.rear_left.get_position(),
				self.rear_right.get_position()
			],
			pose)

	def drive(self, x_speed: float, y_speed: float, rot: float, field_relative: bool):
		"""
		Method to drive the robot using joystick info.

		Args:
			x_speed: Speed of the robot in the x direction (forward).
			y_speed: Speed of the robot in the y direction (sideways).
			rot: Angular rate of the robot.
			field_relative: Whether the provided x and y speeds are relative to the field.
		"""
		# Convert the commanded speeds into the correct units for the drivetrain
		x_speed_delivered = x_speed * DriveConstants.k_max_speed_meters_per_second
		y_speed_delivered = y_speed * DriveConstants.k_max_speed_meters_per_second
		rot_delivered = rot * DriveConstants.k_max_angular_speed

		swerve_module_states = DriveConstants.k_drive_kinematics.to_swerve_module_states(
			ChassisSpeeds.from_field_relative_speeds(
				x_speed_delivered, y_speed_delivered, rot_delivered,
				Rotation2d.from_degrees(self.gyro.get_angle()))
			if field_relative
			else ChassisSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered))
		
		SwerveDriveKinematics.desaturate_wheel_speeds(
			swerve_module_states, DriveConstants.k_max_speed_meters_per_second)
		
		self.front_left.set_desired_state(swerve_module_states[0])
		self.front_right.set_desired_state(swerve_module_states[1])
		self.rear_left.set_desired_state(swerve_module_states[2])
		self.rear_right.set_desired_state(swerve_module_states[3])

	def set_x(self):
		"""Sets the wheels into an X formation to prevent movement."""
		self.front_left.set_desired_state(SwerveModuleState(0, Rotation2d.from_degrees(45)))
		self.front_right.set_desired_state(SwerveModuleState(0, Rotation2d.from_degrees(-45)))
		self.rear_left.set_desired_state(SwerveModuleState(0, Rotation2d.from_degrees(-45)))
		self.rear_right.set_desired_state(SwerveModuleState(0, Rotation2d.from_degrees(45)))

	def set_module_states(self, desired_states: list):
		"""
		Sets the swerve ModuleStates.

		Args:
			desired_states: The desired SwerveModule states.
		"""
		SwerveDriveKinematics.desaturate_wheel_speeds(
			desired_states, DriveConstants.k_max_speed_meters_per_second)
		self.front_left.set_desired_state(desired_states[0])
		self.front_right.set_desired_state(desired_states[1])
		self.rear_left.set_desired_state(desired_states[2])
		self.rear_right.set_desired_state(desired_states[3])

	def reset_encoders(self):
		"""Resets the drive encoders to currently read a position of 0."""
		self.front_left.reset_encoders()
		self.rear_left.reset_encoders()
		self.front_right.reset_encoders()
		self.rear_right.reset_encoders()

	def zero_heading(self):
		"""Zeroes the heading of the robot."""
		self.gyro.reset()

	def get_heading(self) -> float:
		"""
		Returns the heading of the robot.

		Returns:
			the robot's heading in degrees, from -180 to 180
		"""
		return Rotation2d.from_degrees(self.gyro.get_angle()).degrees()

	def get_turn_rate(self) -> float:
		"""
		Returns the turn rate of the robot.

		Returns:
			The turn rate of the robot, in degrees per second
		"""
		return self.gyro.get_rate() * (-1.0 if DriveConstants.k_gyro_reversed else 1.0)
