import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode
from navx import AHRS
from networktables import NetworkTables

class Drive:
	def __init__(self):
		# Motor setup
		self.left_front = WPI_TalonSRX(1)
		self.left_rear = WPI_TalonSRX(4)
		self.right_front = WPI_TalonSRX(3)
		self.right_rear = WPI_TalonSRX(2)

		self.left_front.setInverted(True)
		self.left_rear.setInverted(True)

		# Set all motors to brake mode
		self.left_front.setNeutralMode(NeutralMode.Brake)
		self.left_rear.setNeutralMode(NeutralMode.Brake)
		self.right_front.setNeutralMode(NeutralMode.Brake)
		self.right_rear.setNeutralMode(NeutralMode.Brake)

		# Encoder configuration
		self.WHEEL_DIAMETER_MM = 152.4
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.14159
		self.ENCODER_CPR = 2048

		self.left_encoder = wpilib.Encoder(1, 2)
		self.right_encoder = wpilib.Encoder(3, 4)
		dist_per_pulse = self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR
		self.left_encoder.setDistancePerPulse(dist_per_pulse)
		self.right_encoder.setDistancePerPulse(dist_per_pulse)

		# Create the NavX sensor
		self.navx = AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k200Hz)

		# Travel state
		self.target_distance = 0
		self.overshoot_distance = 300  # Additional travel distance in mm
		self.traveling = False
		self.turning = False
		self.turn_target = 0

	def reset_encoders(self):
		""" Resets encoder values. """
		self.left_encoder.reset()
		self.right_encoder.reset()

	def start_travel(self, distance):
		""" Resets encoders and sets the target travel distance. """
		self.reset_encoders()
		self.target_distance = distance - self.overshoot_distance
		self.traveling = True

	def set_motors(self, left_speed, right_speed):
		""" Sets motor speeds. """
		self.left_front.set(left_speed)
		self.left_rear.set(left_speed)
		self.right_front.set(right_speed)
		self.right_rear.set(right_speed)

	def travel_distance(self):
		""" Drives the robot to a target distance using encoder feedback. """
		if not self.traveling:
			return False

		# Determine travel direction and base speed
		direction = 1 if self.target_distance > 0 else -1
		base_speed = 0.4 * direction

		# Get distances (absolute values)
		left_dist = abs(self.left_encoder.getDistance())
		right_dist = abs(self.right_encoder.getDistance())
		avg_dist = (left_dist + right_dist) / 2

		# Simple correction
		diff = left_dist - right_dist
		correction = 0.0001 * diff
		left_speed = base_speed + correction
		right_speed = base_speed - correction

		# Continue moving if target not yet reached
		if avg_dist < abs(self.target_distance):
			self.set_motors(left_speed, right_speed)
			return True
		else:
			self.set_motors(0, 0)
			self.traveling = False
			return False

	def start_turn(self, degrees, base_speed=0.4):
		""" Initiates a turn by setting a target heading. """
		self.turning = True
		self.turn_target = self.navx.getFusedHeading() + degrees
		while self.turn_target > 360:
			self.turn_target -= 360
		while self.turn_target < 0:
			self.turn_target += 360
		self.turn_base_speed = base_speed if degrees > 0 else -base_speed

	def update_turn(self):
		""" Continuously updates turning motion until target heading is reached. """
		if not self.turning:
			return False

		current_heading = self.navx.getFusedHeading()
		error = self.turn_target - current_heading
		while error > 180:
			error -= 360
		while error < -180:
			error += 360

		if abs(error) < 2:  # Within acceptable margin
			self.set_motors(0, 0)
			self.turning = False
			return False

		kp_turn = 0.01  # Tunable gain
		turn_speed = kp_turn * error
		if turn_speed > abs(self.turn_base_speed):
			turn_speed = abs(self.turn_base_speed)
		elif turn_speed < -abs(self.turn_base_speed):
			turn_speed = -abs(self.turn_base_speed)

		self.set_motors(-turn_speed, turn_speed)
		return True
