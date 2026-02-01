import math
import wpilib
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds
from wpimath.geometry import Rotation2d, Translation2d


class SwerveModule:
	"""A single swerve module with drive and turn motors."""

	def __init__(self, drive_motor_id: int, turn_motor_id: int, absolute_encoder_channel: int):
		"""
		Initialize a swerve module.

		Args:
			drive_motor_id: CAN ID of the drive motor
			turn_motor_id: CAN ID of the turn motor
			absolute_encoder_channel: Analog input channel for absolute encoder
		"""
		self.drive_motor = wpilib.Spark(drive_motor_id)
		self.turn_motor = wpilib.Spark(turn_motor_id)
		self.absolute_encoder = wpilib.AnalogInput(absolute_encoder_channel)

		# Store current state
		self.current_state = SwerveModuleState()

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
