import math
import wpilib
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from rev import CANSparkMax, CANSparkMaxLowLevel


class SwerveModule:
	"""A single swerve module with drive and turn motors."""

	def __init__(self, drive_motor_id: int, turn_motor_id: int, absolute_encoder_channel: int, wheel_encoder_channel: int = None, wheel_diameter_m: float = 0.1):
		"""
		Initialize a swerve module.

		Args:
			drive_motor_id: CAN ID of the drive motor
			turn_motor_id: CAN ID of the turn motor
			absolute_encoder_channel: Analog input channel for absolute encoder
			wheel_encoder_channel: DIO channel for wheel encoder (optional)
			wheel_diameter_m: Diameter of the wheel in meters
		"""
		self.drive_motor = wpilib.Spark(drive_motor_id)
		self.turn_motor = wpilib.Spark(turn_motor_id)
		self.absolute_encoder = wpilib.AnalogInput(absolute_encoder_channel)

		# Motor encoders (built-in to NEO 1650)
		# Note: These would need to be accessed via CANSparkMax if using that driver
		self.motor_encoder = None
		
		# Wheel encoder (separate EasySwerve encoder)
		self.wheel_encoder = None
		if wheel_encoder_channel is not None:
			self.wheel_encoder = wpilib.Encoder(wheel_encoder_channel, wheel_encoder_channel + 1)
			self.wheel_encoder.reset()

		self.wheel_diameter_m = wheel_diameter_m
		self.wheel_circumference_m = math.pi * wheel_diameter_m
		
		# Store current state
		self.current_state = SwerveModuleState()
		self.last_encoder_position = 0.0

	def set_desired_state(self, desired_state: SwerveModuleState):
		"""
		Set the desired state for this module.

		Args:
			desired_state: Desired speed and angle
		"""
		# Optimize the state to avoid spinning more than 90 degrees
		optimized_state = SwerveModuleState.optimize(desired_state, self.get_angle())

		self.drive_motor.set(optimized_state.speed)
		self.turn_motor.set(self._calculate_turn_power(optimized_state.angle))

		self.current_state = optimized_state

	def get_angle(self) -> Rotation2d:
		"""Get the current wheel angle from the absolute encoder."""
		# Convert analog voltage (0-5V) to angle (0-360 degrees)
		voltage = self.absolute_encoder.getAverageVoltage()
		angle_degrees = (voltage / 5.0) * 360.0
		return Rotation2d.fromDegrees(angle_degrees)

	def get_distance_traveled(self) -> float:
		"""
		Get the distance traveled by the wheel using the wheel encoder.
		
		Returns:
			Distance in meters
		"""
		if self.wheel_encoder is None:
			return 0.0
		
		# Get encoder counts and convert to distance
		counts = self.wheel_encoder.get()
		distance = counts * self.wheel_circumference_m / 360.0  # Adjust divisor based on your encoder's counts per revolution
		return distance

	def get_speed(self) -> float:
		"""
		Get the current wheel speed using the wheel encoder.
		
		Returns:
			Speed in meters per second
		"""
		if self.wheel_encoder is None:
			return 0.0
		
		return self.wheel_encoder.getRate() * self.wheel_circumference_m / 360.0

	def reset_encoders(self):
		"""Reset all encoders to zero."""
		if self.wheel_encoder is not None:
			self.wheel_encoder.reset()
		self.last_encoder_position = 0.0

	def _calculate_turn_power(self, target_angle: Rotation2d) -> float:
		"""
		Calculate the power needed to rotate to the target angle.
		Simple proportional control.

		Args:
			target_angle: Target rotation angle

		Returns:
			Power value from -1 to 1
		"""
		current_angle = self.get_angle()
		angle_diff = target_angle.degrees() - current_angle.degrees()

		# Normalize to -180 to 180 degrees
		while angle_diff > 180:
			angle_diff -= 360
		while angle_diff < -180:
			angle_diff += 360

		# Simple proportional control (tune the 0.01 coefficient as needed)
		power = max(-1.0, min(1.0, angle_diff * 0.01))
		return power


class SwerveDrive:
	"""Controls a 4-wheel swerve drive with Rev EasySwerve modules."""

	def __init__(
		self,
		front_left: SwerveModule,
		front_right: SwerveModule,
		rear_left: SwerveModule,
		rear_right: SwerveModule,
		track_width_m: float = 0.5,
		wheelbase_m: float = 0.5,
	):
		"""
		Initialize the swerve drive.

		Args:
			front_left: Front left swerve module
			front_right: Front right swerve module
			rear_left: Rear left swerve module
			rear_right: Rear right swerve module
			track_width_m: Distance between left and right wheels in meters
			wheelbase_m: Distance between front and rear wheels in meters
		"""
		self.front_left = front_left
		self.front_right = front_right
		self.rear_left = rear_left
		self.rear_right = rear_right

		# Create kinematics object
		self.kinematics = SwerveDrive4Kinematics(
			Translation2d(wheelbase_m / 2, track_width_m / 2),  # Front left
			Translation2d(wheelbase_m / 2, -track_width_m / 2),  # Front right
			Translation2d(-wheelbase_m / 2, track_width_m / 2),  # Rear left
			Translation2d(-wheelbase_m / 2, -track_width_m / 2),  # Rear right
		)

		self.robot_angle = Rotation2d()
		
		# Odometry tracking
		self.odometry = SwerveDriveOdometry(
			self.kinematics,
			self.robot_angle,
			[
				SwerveModulePosition(),
				SwerveModulePosition(),
				SwerveModulePosition(),
				SwerveModulePosition(),
			],
			Pose2d(),  # Starting position
		)

	def drive(self, forward_speed: float, strafe_speed: float, rotation_speed: float, field_relative: bool = False):
		"""
		Drive the robot with given speeds.

		Args:
			forward_speed: Forward velocity (-1 to 1)
			strafe_speed: Strafe velocity (-1 to 1)
			rotation_speed: Rotation velocity (-1 to 1)
			field_relative: If True, use field-centric drive. If False, use robot-centric.
		"""
		# Convert percentage inputs to m/s (tune these max speeds as needed)
		max_speed = 4.0  # meters per second
		max_angular = 2 * math.pi  # radians per second

		forward_mps = forward_speed * max_speed
		strafe_mps = strafe_speed * max_speed
		rotation_rps = rotation_speed * max_angular

		# Create chassis speeds
		if field_relative:
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				forward_mps, strafe_mps, rotation_rps, self.robot_angle
			)
		else:
			speeds = ChassisSpeeds(forward_mps, strafe_mps, rotation_rps)

		# Convert chassis speeds to module states
		module_states = self.kinematics.toSwerveModuleStates(speeds)

		# Normalize speeds if any exceed maximum
		SwerveDrive4Kinematics.desaturateWheelSpeeds(module_states, max_speed)

		# Set each module to its desired state
		self.front_left.set_desired_state(module_states[0])
		self.front_right.set_desired_state(module_states[1])
		self.rear_left.set_desired_state(module_states[2])
		self.rear_right.set_desired_state(module_states[3])

	def stop(self):
		"""Stop all motors."""
		self.drive(0, 0, 0)

	def set_robot_angle(self, angle: Rotation2d):
		"""
		Set the robot's current angle (usually from a gyro).

		Args:
			angle: Current robot angle
		"""
		self.robot_angle = angle

	def get_module_angles(self) -> tuple[Rotation2d, Rotation2d, Rotation2d, Rotation2d]:
		"""Get the current angles of all four modules."""
		return (
			self.front_left.get_angle(),
			self.front_right.get_angle(),
			self.rear_left.get_angle(),
			self.rear_right.get_angle(),
		)

	def update_odometry(self):
		"""Update the odometry with current encoder and gyro data."""
		module_positions = [
			SwerveModulePosition(self.front_left.get_distance_traveled(), self.front_left.get_angle()),
			SwerveModulePosition(self.front_right.get_distance_traveled(), self.front_right.get_angle()),
			SwerveModulePosition(self.rear_left.get_distance_traveled(), self.rear_left.get_angle()),
			SwerveModulePosition(self.rear_right.get_distance_traveled(), self.rear_right.get_angle()),
		]
		
		self.odometry.update(self.robot_angle, module_positions)

	def get_pose(self) -> Pose2d:
		"""
		Get the current estimated position and angle of the robot.
		
		Returns:
			Current pose (x, y, angle)
		"""
		return self.odometry.getPose()

	def reset_odometry(self, pose: Pose2d = None):
		"""
		Reset odometry to a known position.
		
		Args:
			pose: The new pose to set. If None, resets to origin.
		"""
		if pose is None:
			pose = Pose2d()
		
		self.odometry.resetPosition(
			self.robot_angle,
			[
				SwerveModulePosition(),
				SwerveModulePosition(),
				SwerveModulePosition(),
				SwerveModulePosition(),
			],
			pose,
		)


class SwerveModulePosition:
	"""Represents the position of a swerve module (distance and angle)."""
	
	def __init__(self, distance_m: float = 0.0, angle: Rotation2d = None):
		"""
		Initialize module position.
		
		Args:
			distance_m: Distance traveled in meters
			angle: Wheel angle
		"""
		self.distance = distance_m
		self.angle = angle if angle is not None else Rotation2d()


class SwerveDriveOdometry:
	"""Tracks the robot's position and orientation using swerve module data."""
	
	def __init__(self, kinematics: SwerveDrive4Kinematics, gyro_angle: Rotation2d, module_positions: list, initial_pose: Pose2d = None):
		"""
		Initialize odometry.
		
		Args:
			kinematics: Swerve drive kinematics
			gyro_angle: Current robot angle from gyro
			module_positions: List of SwerveModulePosition objects
			initial_pose: Starting position (default: origin)
		"""
		self.kinematics = kinematics
		self.gyro_angle = gyro_angle
		self.module_positions = module_positions
		self.pose = initial_pose if initial_pose is not None else Pose2d()
		self.previous_angle = gyro_angle

	def update(self, gyro_angle: Rotation2d, module_positions: list):
		"""
		Update odometry with new sensor data.
		
		Args:
			gyro_angle: Current angle from gyro
			module_positions: List of SwerveModulePosition objects
		"""
		# Calculate the change in angle
		angle_delta = gyro_angle.radians() - self.previous_angle.radians()
		self.previous_angle = gyro_angle
		
		# Calculate chassis speeds from module states
		module_speeds = ChassisSpeeds()
		if len(module_positions) >= 4:
			module_speeds = self.kinematics.toChassisSpeeds(
				SwerveModuleState(module_positions[0].distance, module_positions[0].angle),
				SwerveModuleState(module_positions[1].distance, module_positions[1].angle),
				SwerveModuleState(module_positions[2].distance, module_positions[2].angle),
				SwerveModuleState(module_positions[3].distance, module_positions[3].angle),
			)
		
		# Update position
		current_x = self.pose.X()
		current_y = self.pose.Y()
		
		# Convert to field-relative movement
		cos_angle = math.cos(gyro_angle.radians())
		sin_angle = math.sin(gyro_angle.radians())
		
		dx = module_speeds.vx * cos_angle - module_speeds.vy * sin_angle
		dy = module_speeds.vx * sin_angle + module_speeds.vy * cos_angle
		
		# Update pose
		self.pose = Pose2d(current_x + dx, current_y + dy, gyro_angle)
		self.module_positions = module_positions
		self.gyro_angle = gyro_angle

	def getPose(self) -> Pose2d:
		"""Get the current robot pose."""
		return self.pose

	def resetPosition(self, gyro_angle: Rotation2d, module_positions: list, new_pose: Pose2d):
		"""
		Reset odometry to a new pose.
		
		Args:
			gyro_angle: Current gyro angle
			module_positions: Current module positions
			new_pose: New pose to set
		"""
		self.pose = new_pose
		self.gyro_angle = gyro_angle
		self.previous_angle = gyro_angle
		self.module_positions = module_positions
