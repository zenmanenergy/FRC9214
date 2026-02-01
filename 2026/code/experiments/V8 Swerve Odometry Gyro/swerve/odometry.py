import math
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.geometry import Rotation2d
from .module_position import SwerveModulePosition
from .gyro import NavXGyro


class SwerveDriveOdometry:
	"""Tracks the robot's position and orientation using swerve module data with optional sensor fusion."""
	
	def __init__(self, kinematics, gyro_angle: Rotation2d, module_positions: list, initial_pose=None, gyro: NavXGyro = None):
		"""
		Initialize odometry with optional sensor fusion.
		
		Args:
			kinematics: Swerve drive kinematics
			gyro_angle: Current robot angle from gyro
			module_positions: List of SwerveModulePosition objects
			initial_pose: Starting position (default: origin)
			gyro: NavXGyro instance for sensor fusion
		"""
		from wpimath.geometry import Pose2d
		
		self.kinematics = kinematics
		self.gyro_angle = gyro_angle
		self.module_positions = module_positions
		self.pose = initial_pose if initial_pose is not None else Pose2d()
		self.previous_angle = gyro_angle
		self.gyro = gyro
		
		# Kalman filter parameters for sensor fusion
		# These tune how much weight we give to gyro vs encoder estimates
		self.gyro_trust = 0.95  # How much we trust the gyro for angle (0-1)
		self.encoder_trust = 0.05  # How much we trust encoder angle estimates
		
		# Track previous module distances for velocity estimation
		self.previous_distances = [0.0, 0.0, 0.0, 0.0]

	def update(self, gyro_angle: Rotation2d, module_positions: list):
		"""
		Update odometry with new sensor data using sensor fusion.
		
		Args:
			gyro_angle: Current angle from gyro
			module_positions: List of SwerveModulePosition objects
		"""
		from wpimath.geometry import Pose2d
		
		# Calculate encoder-based angle estimate (from module directions)
		encoder_angle = self._estimate_angle_from_modules(module_positions)
		
		# Sensor fusion: blend gyro and encoder angle estimates
		if self.gyro is not None and self.gyro.is_ready():
			# Weighted blend: favor gyro which is more accurate for rotation
			fused_angle_rad = (
				self.gyro_trust * gyro_angle.radians() +
				self.encoder_trust * encoder_angle.radians()
			)
			fused_angle = Rotation2d(fused_angle_rad)
		else:
			# No gyro available, use gyro_angle directly
			fused_angle = gyro_angle
		
		# Calculate the change in angle
		angle_delta = fused_angle.radians() - self.previous_angle.radians()
		self.previous_angle = fused_angle
		
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
		
		# Convert to field-relative movement using fused angle
		cos_angle = math.cos(fused_angle.radians())
		sin_angle = math.sin(fused_angle.radians())
		
		dx = module_speeds.vx * cos_angle - module_speeds.vy * sin_angle
		dy = module_speeds.vx * sin_angle + module_speeds.vy * cos_angle
		
		# Update pose
		self.pose = Pose2d(current_x + dx, current_y + dy, fused_angle)
		self.module_positions = module_positions
		self.gyro_angle = fused_angle

	def _estimate_angle_from_modules(self, module_positions: list) -> Rotation2d:
		"""
		Estimate robot angle from swerve module directions.
		
		Args:
			module_positions: List of SwerveModulePosition objects
			
		Returns:
			Estimated angle from module directions
		"""
		if len(module_positions) < 4:
			return Rotation2d()
		
		# Average the module angles
		angle_sum = (
			module_positions[0].angle.radians() +
			module_positions[1].angle.radians() +
			module_positions[2].angle.radians() +
			module_positions[3].angle.radians()
		) / 4.0
		
		return Rotation2d(angle_sum)

	def getPose(self):
		"""Get the current robot pose."""
		return self.pose

	def resetPosition(self, gyro_angle: Rotation2d, module_positions: list, new_pose):
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

	def set_gyro_trust(self, trust_factor: float):
		"""
		Set how much trust to give the gyro vs encoders.
		
		Args:
			trust_factor: 0-1, higher values trust gyro more
		"""
		self.gyro_trust = max(0.0, min(1.0, trust_factor))
		self.encoder_trust = 1.0 - self.gyro_trust

	def set_gyro(self, gyro: NavXGyro):
		"""Update the gyro instance."""
		self.gyro = gyro
