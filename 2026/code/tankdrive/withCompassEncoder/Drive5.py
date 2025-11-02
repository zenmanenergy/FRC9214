import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode
from wpilib.controller import PIDController

class Drive:
	def __init__(self, DriveJoystick, left_front, left_rear, right_front, right_rear, left_encoder, right_encoder):
		self.DriveJoystick = DriveJoystick
		self.left_front = left_front
		self.left_rear = left_rear
		self.right_front = right_front
		self.right_rear = right_rear
		self.leftEncoder = left_encoder
		self.rightEncoder = right_encoder

		# Encoder configuration
		self.WHEEL_DIAMETER_MM = 152.4
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.14159
		self.ENCODER_CPR = 2048
		dist_per_pulse = self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR

		self.leftEncoder.setDistancePerPulse(dist_per_pulse)
		self.rightEncoder.setDistancePerPulse(dist_per_pulse)

		# Travel state
		self.overshoot_distance = 300  # Overshoot correction
		self.traveling = False
		self.target_distance = 0

		# PID Controller for Straight-Line Correction
		self.kP = 0.01   # Proportional gain (adjust if needed)
		self.kI = 0.0001 # Integral gain (prevents drift over time)
		self.kD = 0.001  # Derivative gain (reduces sudden corrections)

		self.pid = PIDController(self.kP, self.kI, self.kD)
		self.pid.setSetpoint(0)  # Setpoint is zero error (left == right)

	def reset(self):
		"""Stops movement and resets travel state."""
		self.traveling = False
		self.target_distance = 0
		self.set_motors(0, 0)

	def reset_encoders(self):
		"""Resets encoder values."""
		self.leftEncoder.reset()
		self.rightEncoder.reset()

	def set_motors(self, left_speed, right_speed):
		"""Sets motor speeds."""
		self.left_front.set(left_speed)
		self.left_rear.set(left_speed)
		self.right_front.set(right_speed)
		self.right_rear.set(right_speed)

	def start_travel(self, distance):
		"""Resets encoders and begins moving toward the target distance."""
		print('start_travel')
		self.reset_encoders()

		# Ensure overshoot correction is applied correctly
		if distance > 0:
			self.target_distance = distance + self.overshoot_distance
		else:
			self.target_distance = distance - self.overshoot_distance

		self.traveling = True

	def update_travel(self, base_speed=0.4):
		"""Updates movement toward target distance with PID straight-line correction."""
		if not self.traveling:
			return False

		# Determine travel direction
		direction = 1 if self.target_distance > 0 else -1
		base_speed = base_speed * direction

		# Get encoder distances
		left_dist = self.leftEncoder.getDistance()
		right_dist = self.rightEncoder.getDistance()
		avg_dist = (abs(left_dist) + abs(right_dist)) / 2.0

		# PID correction based on encoder difference
		error = left_dist - right_dist
		correction = self.pid.calculate(error)

		# Compute motor speeds with correction
		left_speed = base_speed - correction
		right_speed = base_speed + correction

		# Limit speeds (ensure motors don't exceed safe limits)
		left_speed = max(min(left_speed, 1.0), -1.0)
		right_speed = max(min(right_speed, 1.0), -1.0)

		# Continue moving if the target distance hasn't been reached.
		if avg_dist < abs(self.target_distance):
			self.set_motors(left_speed, right_speed)
			return True
		else:
			self.set_motors(0, 0)
			self.traveling = False
			return False
