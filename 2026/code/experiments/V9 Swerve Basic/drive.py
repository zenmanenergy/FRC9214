import math
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds
from wpimath.geometry import Rotation2d, Translation2d

# Import rev - only available on RoboRIO (ARM Linux), not on Windows
try:
	import rev
	HAS_REV = True
except (ImportError, ModuleNotFoundError):
	HAS_REV = False
	rev = None


class SwerveModule:
	"""A single swerve module with REV drive and turn motors."""

	def __init__(self, drive_motor_id: int, turn_motor_id: int, encoder_offset: float = 0.0):
		"""Initialize a swerve module with REV SparkMax motors."""
		self.drive_motor_id = drive_motor_id
		self.turn_motor_id = turn_motor_id
		
		# Conversion factors
		self.WHEEL_DIAMETER_M = 0.1016
		self.GEAR_RATIO = 6.75
		self.drive_conversion = math.pi * self.WHEEL_DIAMETER_M / self.GEAR_RATIO
		self.turn_conversion = 360.0
		
		# Initialize as None for sim
		self.drive_motor = None
		self.turn_motor = None
		self.drive_encoder = None
		self.turn_absolute_encoder = None
		self.current_state = SwerveModuleState()
		
		# Only initialize actual hardware on RoboRIO
		if not HAS_REV:
			return

		try:
			self.drive_motor = rev.SparkMax(drive_motor_id, rev.SparkMax.MotorType.kBrushless)
			self.turn_motor = rev.SparkMax(turn_motor_id, rev.SparkMax.MotorType.kBrushless)
			
			# Try to set inverted on drive motor (might not exist in this API)
			try:
				self.drive_motor.setInverted(False)
			except Exception:
				pass
			
			self.drive_encoder = self.drive_motor.getEncoder()
			self.turn_absolute_encoder = self.turn_motor.getAbsoluteEncoder()
			
			print("M%d INIT: Drive=%d Turn=%d" % (drive_motor_id, drive_motor_id, turn_motor_id))
		except Exception as e:
			print("M%d ERROR: %s" % (drive_motor_id, e))
			self.drive_motor = None
			self.turn_motor = None

	def set_desired_state(self, desired_state: SwerveModuleState):
		"""Set the desired state for this module."""
		if self.drive_motor is None:
			return

		if desired_state is None:
			return

		# Implement our own optimization to handle reverse motion
		# If the desired angle is ~180° different from current, flip speed and rotate 180°
		optimized_state = desired_state
		try:
			# Use actual current angle from encoder, not the commanded state
			current_angle = self.get_angle()
			desired_angle = desired_state.angle
			
			# Calculate angle difference
			delta = desired_angle.degrees() - current_angle.degrees()
			
			# Normalize to [-180, 180]
			while delta > 180:
				delta -= 360
			while delta < -180:
				delta += 360
			
			# If angle change > 90°, flip speed and rotate other way
			if abs(delta) > 90:
				optimized_state = SwerveModuleState(
					-desired_state.speed,
					desired_angle.rotateBy(Rotation2d.fromDegrees(180))
				)
		except Exception:
			# If anything fails, just use the desired state
			pass

		# Set drive motor
		try:
			speed_cmd = optimized_state.speed
			
			# Clamp to [-1.0, 1.0] for motor.set()
			speed_cmd = max(-1.0, min(1.0, speed_cmd))
			
			# Set the motor first
			self.drive_motor.set(speed_cmd)
			
			# Only print when there's actual motion
			if abs(speed_cmd) > 0.01:
				actual_output = self.drive_motor.getAppliedOutput()
				print("M%d: Speed=%.3f Applied=%.3f" % (self.drive_motor_id, speed_cmd, actual_output))
			
		except Exception as e:
			print("M%d ERROR: %s" % (self.drive_motor_id, e))

		# Control turn motor to reach desired angle
		try:
			if self.turn_motor is not None:
				current_angle = self.get_angle()
				desired_angle = optimized_state.angle
				
				# Calculate shortest path to desired angle
				angle_error = desired_angle.degrees() - current_angle.degrees()
				
				# Normalize to [-180, 180]
				while angle_error > 180:
					angle_error -= 360
				while angle_error < -180:
					angle_error += 360
				
				# Proportional control with stronger gain to overcome friction
				# Use 2.0 multiplier so 90 degrees = full power (was too weak at 0.167)
				turn_cmd = angle_error / 180.0 * 2.0
				
				# Clamp to [-1.0, 1.0]
				turn_cmd = max(-1.0, min(1.0, turn_cmd))
				
				if abs(turn_cmd) > 0.01:
					print("M%d TURN: Error=%.1f° Cmd=%.3f" % (self.turn_motor_id, angle_error, turn_cmd))
				
				self.turn_motor.set(turn_cmd)
		except Exception:
			pass

		# Store the OPTIMIZED state (represents what we're trying to command)
		# Note: The actual module state may differ due to motor response lag
		self.current_state = optimized_state

	def stop(self):
		"""Stop both motors immediately."""
		if self.drive_motor is None:
			return
		try:
			self.drive_motor.set(0.0)
			self.turn_motor.set(0.0)
			self.current_state = SwerveModuleState()
		except Exception:
			pass

	def get_angle(self) -> Rotation2d:
		"""Get the current wheel angle from the absolute encoder."""
		if self.turn_absolute_encoder is None:
			return Rotation2d()
		
		try:
			encoder_rotations = self.turn_absolute_encoder.getPosition()
			angle_degrees = encoder_rotations * self.turn_conversion
			return Rotation2d.fromDegrees(angle_degrees)
		except Exception:
			return Rotation2d()


class SwerveDrive:
	"""Controls a 4-wheel swerve drive with Rev SparkMax modules."""

	def __init__(
		self,
		front_left: SwerveModule,
		front_right: SwerveModule,
		rear_left: SwerveModule,
		rear_right: SwerveModule,
		track_width_m: float = 0.5,
		wheelbase_m: float = 0.5,
	):
		"""Initialize the swerve drive."""
		self.front_left = front_left
		self.front_right = front_right
		self.rear_left = rear_left
		self.rear_right = rear_right

		# Create kinematics object
		self.kinematics = SwerveDrive4Kinematics(
			Translation2d(wheelbase_m / 2, track_width_m / 2),   # Front left
			Translation2d(wheelbase_m / 2, -track_width_m / 2),  # Front right
			Translation2d(-wheelbase_m / 2, track_width_m / 2),  # Rear left
			Translation2d(-wheelbase_m / 2, -track_width_m / 2), # Rear right
		)

		self.robot_angle = Rotation2d()

	def drive(self, forward_speed: float, strafe_speed: float, rotation_speed: float, field_relative: bool = False):
		"""Drive the robot with given speeds."""
		# Convert percentage inputs to m/s
		max_speed = 4.0
		max_angular = 2 * math.pi

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

		# Convert to module states
		module_states = self.kinematics.toSwerveModuleStates(speeds)

		# Normalize if needed
		SwerveDrive4Kinematics.desaturateWheelSpeeds(module_states, max_speed)

		# Send to modules
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
