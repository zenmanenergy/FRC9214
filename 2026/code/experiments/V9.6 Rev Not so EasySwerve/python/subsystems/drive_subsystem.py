# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition, SwerveModuleState
from wpilib import SPI
try:
	from navx import AHRS
	HAS_NAVX = True
except (ImportError, ModuleNotFoundError):
	HAS_NAVX = False
	AHRS = None
from constants import DriveConstants
from subsystems.easy_swerve_module import EasySwerveModule


class DriveSubsystem:
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
			DriveConstants.k_front_left_turning_motor_on_bottom,
			"Front Left",
			DriveConstants.k_front_left_encoder_offset)

		self.front_right = EasySwerveModule(
			DriveConstants.k_front_right_driving_can_id,
			DriveConstants.k_front_right_turning_can_id,
			DriveConstants.k_front_right_chassis_angular_offset,
			DriveConstants.k_front_right_driving_motor_on_bottom,
			DriveConstants.k_front_right_turning_motor_on_bottom,
			"Front Right",
			DriveConstants.k_front_right_encoder_offset)

		self.rear_left = EasySwerveModule(
			DriveConstants.k_rear_left_driving_can_id,
			DriveConstants.k_rear_left_turning_can_id,
			DriveConstants.k_rear_left_chassis_angular_offset,
			DriveConstants.k_rear_left_driving_motor_on_bottom,
			DriveConstants.k_rear_left_turning_motor_on_bottom,
			"Rear Left",
			DriveConstants.k_rear_left_encoder_offset)

		self.rear_right = EasySwerveModule(
			DriveConstants.k_rear_right_driving_can_id,
			DriveConstants.k_rear_right_turning_can_id,
			DriveConstants.k_rear_right_chassis_angular_offset,
			DriveConstants.k_rear_right_driving_motor_on_bottom,
			DriveConstants.k_rear_right_turning_motor_on_bottom,
			"Rear Right",
			DriveConstants.k_rear_right_encoder_offset)

		# Log module initialization status
		print("\n[INIT] Module Status:")
		print(f"  Front Left:  Turn spark={self.front_left.turning_spark is not None}")
		print(f"  Front Right: Turn spark={self.front_right.turning_spark is not None}")
		print(f"  Rear Left:   Turn spark={self.rear_left.turning_spark is not None}")
		print(f"  Rear Right:  Turn spark={self.rear_right.turning_spark is not None}")
		print()

		# The gyro sensor (NavX2 via SPI)
		self.gyro = None
		if HAS_NAVX:
			try:
				self.gyro = AHRS(SPI.Port.kMXP)
				print("[INIT] NavX2 gyro initialized successfully")
			except Exception as e:
				print(f"[WARNING] Failed to initialize NavX2 gyro: {e}")
				self.gyro = None
		else:
			print("[WARNING] NavX library not available")

		# Odometry class for tracking robot pose
		gyro_angle = Rotation2d.fromDegrees(self.gyro.getAngle() if self.gyro is not None else 0)
		self.odometry = SwerveDrive4Odometry(
			DriveConstants.k_drive_kinematics,
			gyro_angle,
			[
				self.front_left.get_position(),
				self.front_right.get_position(),
				self.rear_left.get_position(),
				self.rear_right.get_position()
			],
			Pose2d())
		
		# CAN staggering counter - spreads motor commands across multiple cycles
		# This reduces CAN bus saturation and encoder feedback delays
		self.can_stagger_cycle = 0
		self.last_module_states = [
			SwerveModuleState(0, Rotation2d()),
			SwerveModuleState(0, Rotation2d()),
			SwerveModuleState(0, Rotation2d()),
			SwerveModuleState(0, Rotation2d())
		]
		self.cached_module_states = [
			SwerveModuleState(0, Rotation2d()),
			SwerveModuleState(0, Rotation2d()),
			SwerveModuleState(0, Rotation2d()),
			SwerveModuleState(0, Rotation2d())
		]

	def periodic(self):
		"""Update the odometry in the periodic block."""
		gyro_angle = Rotation2d.fromDegrees(self.gyro.getAngle() if self.gyro is not None else 0)
		self.odometry.update(
			gyro_angle,
			[
				self.front_left.get_position(),
				self.front_right.get_position(),
				self.rear_left.get_position(),
				self.rear_right.get_position()
			])
		
		# Apply turn commands to all wheels
		self.front_left.periodic_turn_command()
		self.front_right.periodic_turn_command()
		self.rear_left.periodic_turn_command()
		self.rear_right.periodic_turn_command()

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
		gyro_angle = Rotation2d.fromDegrees(self.gyro.getAngle() if self.gyro is not None else 0)
		self.odometry.reset_position(
			gyro_angle,
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
		Uses staggered updates: Only recalculates kinematics once per 4 cycles (Front Left's turn).

		Args:
			x_speed: Speed of the robot in the x direction (forward).
			y_speed: Speed of the robot in the y direction (sideways).
			rot: Angular rate of the robot.
			field_relative: Whether the provided x and y speeds are relative to the field.
		"""
		current_motor = self.can_stagger_cycle % 4
		
		# Only recalculate kinematics every 4 cycles (when Front Left is due for update)
		# This prevents all 4 motors from getting new targets every cycle
		if current_motor == 0:
			# Recalculate for all motors (Front Left's turn)
			x_speed_delivered = x_speed * DriveConstants.k_max_speed_meters_per_second
			y_speed_delivered = y_speed * DriveConstants.k_max_speed_meters_per_second
			rot_delivered = rot * DriveConstants.k_max_angular_speed

			gyro_angle = Rotation2d.fromDegrees(self.gyro.getAngle() if self.gyro is not None else 0)
			swerve_module_states = DriveConstants.k_drive_kinematics.toSwerveModuleStates(
				ChassisSpeeds.fromFieldRelativeSpeeds(
					x_speed_delivered, y_speed_delivered, rot_delivered,
					gyro_angle)
				if field_relative
				else ChassisSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered))
			
			SwerveDrive4Kinematics.desaturateWheelSpeeds(
				swerve_module_states, DriveConstants.k_max_speed_meters_per_second)
			
			# Cache the newly calculated states
			self.cached_module_states = swerve_module_states
			self.last_module_states = swerve_module_states
		
		# Update only THIS motor's target from cached states
		motor_modules = [self.front_left, self.front_right, self.rear_left, self.rear_right]
		motor_modules[current_motor].set_desired_state(self.cached_module_states[current_motor])
		
		self.can_stagger_cycle += 1

	def set_x(self):
		"""Sets the wheels into an X formation to prevent movement."""
		x_states = [
			SwerveModuleState(0, Rotation2d.from_degrees(45)),
			SwerveModuleState(0, Rotation2d.from_degrees(-45)),
			SwerveModuleState(0, Rotation2d.from_degrees(-45)),
			SwerveModuleState(0, Rotation2d.from_degrees(45))
		]
		self.last_module_states = x_states
		self.cached_module_states = x_states
		
		# Stagger the X formation update - only update THIS motor's target
		current_motor = self.can_stagger_cycle % 4
		motor_modules = [self.front_left, self.front_right, self.rear_left, self.rear_right]
		motor_modules[current_motor].set_desired_state(x_states[current_motor])
		self.can_stagger_cycle += 1

	def set_module_states(self, desired_states: list):
		"""
		Sets the swerve ModuleStates.

		Args:
			desired_states: The desired SwerveModule states.
		"""
		SwerveDrive4Kinematics.desaturateWheelSpeeds(
			desired_states, DriveConstants.k_max_speed_meters_per_second)
		
		self.last_module_states = desired_states
		self.cached_module_states = desired_states
		
		# Stagger the module state updates - only update THIS motor's target
		current_motor = self.can_stagger_cycle % 4
		motor_modules = [self.front_left, self.front_right, self.rear_left, self.rear_right]
		motor_modules[current_motor].set_desired_state(desired_states[current_motor])
		self.can_stagger_cycle += 1

	def reset_encoders(self):
		"""Resets the drive encoders to currently read a position of 0."""
		self.front_left.reset_encoders()
		self.rear_left.reset_encoders()
		self.front_right.reset_encoders()
		self.rear_right.reset_encoders()

	def set_debug_mode(self, enabled: bool):
		"""Enable/disable debug output for all modules."""
		self.front_left.debug_enabled = enabled
		self.front_right.debug_enabled = enabled
		self.rear_left.debug_enabled = enabled
		self.rear_right.debug_enabled = enabled

	def zero_heading(self):
		"""Zeroes the heading of the robot."""
		if self.gyro is not None:
			self.gyro.reset()

	def spin_turn_motors(self, power: float):
		"""
		Spins all turn motors at the given power for testing.
		Simple open-loop control - no PID, no logic, just turn them on.
		
		Args:
			power: Motor power from -1 to 1
		"""
		motors = [
			("Front Left", self.front_left),
			("Front Right", self.front_right),
			("Rear Left", self.rear_left),
			("Rear Right", self.rear_right)
		]
		
		for name, module in motors:
			if module and module.turning_spark:
				# Pure duty cycle control - bypasses all closed-loop
				module.turning_spark.set(power)
				print(f"[SPIN TEST] {name}: {power}")
			else:
				print(f"[SPIN TEST] {name}: NOT INITIALIZED")

	def test_turn_motors_no_encoder(self, power: float):
		"""
		Test turning motors with encoder feedback disabled.
		This helps diagnose if the encoder is causing resistance.
		
		Args:
			power: Motor power from -1 to 1
		"""
		try:
			from rev import SparkMaxConfig, ResetMode, PersistMode
			
			# Create a config that disables all feedback and closed-loop control
			no_feedback_config = SparkMaxConfig()
			no_feedback_config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
			no_feedback_config.smartCurrentLimit(60)  # Increase current limit
			
			motors = [
				("Front Left", self.front_left),
				("Front Right", self.front_right),
				("Rear Left", self.rear_left),
				("Rear Right", self.rear_right)
			]
			
			print("[ENCODER TEST] Testing motors WITHOUT encoder feedback (Coast mode)...")
			for name, module in motors:
				if module and module.turning_spark:
					module.turning_spark.configure(no_feedback_config, ResetMode.kNoResetSafeParameters, PersistMode.kDontPersistParameters)
					module.turning_spark.set(power)
					print(f"[ENCODER TEST] {name}: {power} (coast mode)")
				else:
					print(f"[ENCODER TEST] {name}: NOT INITIALIZED")
		except Exception as e:
			print(f"[ERROR] test_turn_motors_no_encoder failed: {e}")

	def get_heading(self) -> float:
		"""
		Returns the heading of the robot.

		Returns:
			the robot's heading in degrees, from -180 to 180
		"""
		if self.gyro is None:
			return 0.0
		return Rotation2d.fromDegrees(self.gyro.getAngle()).degrees()

	def get_turn_rate(self) -> float:
		"""
		Returns the turn rate of the robot.

		Returns:
			The turn rate of the robot, in degrees per second
		"""
		if self.gyro is None:
			return 0.0
		return self.gyro.getRate() * (-1.0 if DriveConstants.k_gyro_reversed else 1.0)

	def print_all_sparkmax_config(self):
		"""Print SparkMax configuration for CAN ID 5 (Rear Left Turning Motor) only."""
		print("\n" + "="*80)
		print("SPARKMAX CONFIGURATION - CAN ID 5 (REAR LEFT TURNING MOTOR)")
		print("="*80)
		
		self.rear_left.print_sparkmax_config()
		
		print("="*80)
		print("END OF CONFIGURATION DUMP")
		print("="*80 + "\n")
