import math
import wpilib
from wpimath.geometry import Rotation2d


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
		from wpimath.kinematics import SwerveModuleState
		self.current_state = SwerveModuleState()
		self.last_encoder_position = 0.0

	def set_desired_state(self, desired_state):
		"""
		Set the desired state for this module.

		Args:
			desired_state: Desired speed and angle
		"""
		from wpimath.kinematics import SwerveModuleState
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
