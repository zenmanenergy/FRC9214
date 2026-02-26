import math
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from .module import SwerveModule
from .module_position import SwerveModulePosition
from .odometry import SwerveDriveOdometry
from .gyro import NavXGyro


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
		gyro: NavXGyro = None,
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
			gyro: NavXGyro instance for sensor fusion (optional)
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
		self.gyro = gyro
		
		# Odometry tracking with sensor fusion
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
			gyro=gyro,
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
		from wpimath.kinematics import SwerveDrive4Kinematics
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
		# Get gyro angle if available, otherwise use odometry estimate
		if self.gyro is not None and self.gyro.is_ready():
			gyro_angle = self.gyro.get_angle()
		else:
			gyro_angle = self.odometry.getPose().rotation()
		
		module_positions = [
			SwerveModulePosition(self.front_left.get_distance_traveled(), self.front_left.get_angle()),
			SwerveModulePosition(self.front_right.get_distance_traveled(), self.front_right.get_angle()),
			SwerveModulePosition(self.rear_left.get_distance_traveled(), self.rear_left.get_angle()),
			SwerveModulePosition(self.rear_right.get_distance_traveled(), self.rear_right.get_angle()),
		]
		
		self.odometry.update(gyro_angle, module_positions)
		self.robot_angle = self.odometry.getPose().rotation()

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
		
		# Also reset gyro if available
		if self.gyro is not None and self.gyro.is_ready():
			self.gyro.reset()
		
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

	def get_gyro_angle(self) -> Rotation2d:
		"""
		Get the current angle from the gyro.
		
		Returns:
			Gyro angle or zero if not connected
		"""
		if self.gyro is not None and self.gyro.is_ready():
			return self.gyro.get_angle()
		return Rotation2d()

	def calibrate_gyro(self):
		"""Calibrate the gyro (should be called while robot is stationary)."""
		if self.gyro is not None:
			self.gyro.calibrate()

	def is_gyro_ready(self) -> bool:
		"""Check if gyro is ready to use."""
		if self.gyro is not None:
			return self.gyro.is_ready()
		return False
