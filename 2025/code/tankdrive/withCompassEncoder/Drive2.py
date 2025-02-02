import math
import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode
from navx import AHRS

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

		# Coordinate state (in mm)
		self.current_x = 0
		self.current_y = 0
		# These store the starting coordinates for a travel command.
		self.start_x = 0
		self.start_y = 0
		# The heading (in degrees) at the start of travel.
		self.travel_start_heading = 0

		# Navigation state (for moving to a coordinate)
		self.navigating = False
		self.navigation_distance = 0
		self.navigation_started_travel = False

	# --- New Coordinate Methods ---
	def setCoordinates(self, x, y):
		"""Sets the robot's current coordinates (in mm)."""
		self.current_x = x
		self.current_y = y

	def getCoordinates(self):
		"""Returns the robot's current coordinates as a tuple (x, y) in mm."""
		return (self.current_x, self.current_y)

	def reset_encoders(self):
		"""Resets encoder values."""
		self.left_encoder.reset()
		self.right_encoder.reset()

	def start_travel(self, distance):
		"""Resets encoders, stores the starting coordinates and heading, and sets the target travel distance."""
		self.reset_encoders()
		# Save the coordinate where travel starts.
		self.start_x = self.current_x
		self.start_y = self.current_y
		# Record the heading at the start of travel.
		self.travel_start_heading = self.navx.getFusedHeading()
		# Adjust target distance to account for overshoot.
		self.target_distance = distance - self.overshoot_distance
		self.traveling = True

	def set_motors(self, left_speed, right_speed):
		"""Sets motor speeds."""
		self.left_front.set(left_speed)
		self.left_rear.set(left_speed)
		self.right_front.set(right_speed)
		self.right_rear.set(right_speed)

	def travel_distance(self):
		"""Drives the robot toward the target distance using encoder feedback and updates the current coordinates."""
		if not self.traveling:
			return False

		# Determine travel direction and base speed.
		direction = 1 if self.target_distance > 0 else -1
		base_speed = 0.4 * direction

		# Get the average distance traveled (in mm) from the encoders.
		left_dist = abs(self.left_encoder.getDistance())
		right_dist = abs(self.right_encoder.getDistance())
		avg_dist = (left_dist + right_dist) / 2.0

		# Apply a simple correction based on the difference between encoders.
		diff = left_dist - right_dist
		correction = 0.0001 * diff
		left_speed = base_speed + correction
		right_speed = base_speed - correction

		# Update the current coordinates using the travel distance.
		# For backward travel, adjust the heading by 180 degrees.
		effective_heading = self.travel_start_heading if direction > 0 else (self.travel_start_heading + 180) % 360
		heading_rad = math.radians(effective_heading)
		self.current_x = self.start_x + avg_dist * math.cos(heading_rad)
		self.current_y = self.start_y + avg_dist * math.sin(heading_rad)

		# Continue moving if the target distance hasn't been reached.
		if avg_dist < abs(self.target_distance):
			self.set_motors(left_speed, right_speed)
			return True
		else:
			self.set_motors(0, 0)
			self.traveling = False
			return False

	def start_turn(self, degrees, base_speed=0.4):
		"""Initiates a turn by setting a target heading."""
		self.turning = True
		self.turn_target = self.navx.getFusedHeading() + degrees
		while self.turn_target > 360:
			self.turn_target -= 360
		while self.turn_target < 0:
			self.turn_target += 360
		self.turn_base_speed = base_speed if degrees > 0 else -base_speed

	def update_turn(self):
		"""Updates the turning motion until the target heading is reached."""
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

	# The existing navigation functions remain unchanged.
	def navigate_to(self, current_x, current_y, target_x, target_y):
		"""
		Calculates the required turn and travel distance to move from the current
		coordinate to the target coordinate (all in mm) and initiates the movement.
		"""
		dx = target_x - current_x
		dy = target_y - current_y
		distance = math.sqrt(dx**2 + dy**2)
		desired_angle = math.degrees(math.atan2(dy, dx))
		if desired_angle < 0:
			desired_angle += 360

		current_heading = self.navx.getFusedHeading()
		turn_angle = desired_angle - current_heading
		while turn_angle > 180:
			turn_angle -= 360
		while turn_angle < -180:
			turn_angle += 360

		self.start_turn(turn_angle, 0.4)
		self.navigation_distance = distance
		self.navigating = True
		self.navigation_started_travel = False

	def update_navigation(self):
		"""
		Should be called periodically. This function checks if the turn is complete,
		and if so, initiates the travel command toward the target coordinate. It then
		updates the travel command until completion.
		Returns True if still navigating, or False if navigation is complete.
		"""
		if not self.navigating:
			return False

		if not self.turning and not self.navigation_started_travel:
			self.start_travel(self.navigation_distance)
			self.navigation_started_travel = True

		if self.traveling:
			self.traveling = self.travel_distance()
			if not self.traveling:
				self.navigating = False
				self.navigation_started_travel = False

		return self.navigating
