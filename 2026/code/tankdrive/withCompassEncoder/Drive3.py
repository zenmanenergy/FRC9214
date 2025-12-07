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
		self.overshoot_distance = 300  # Additional travel distance in mm
		
		

		

	def reset(self):
		self.traveling = False
		self.turning = False
		self.navigating = False

		self.target_distance = 0
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
		self.navigation_distance = 0
		self.navigation_started_travel = False
		self.set_motors(0,0)

	# --- New Coordinate Methods ---
	def setCoordinates(self, x, y):
		"""Sets the robot's current coordinates (in mm)."""
		self.current_x = x
		self.current_y = y

	def getCoordinates(self):
		"""Returns the robot's current coordinates as a tuple (x, y) in mm."""
		return (self.current_x, self.current_y)

	def getHeading(self):
		"""Returns the robot's current coordinates as a tuple (x, y) in mm."""
		return self.navx.getFusedHeading() 
	# [self.navx.getFusedHeading(),self.navx.getCompassHeading(), self.navx.getYaw(),self.navx.isMagneticDisturbance(),self.navx.isMagnetometerCalibrated()]

	def reset_encoders(self):
		"""Resets encoder values."""
		self.left_encoder.reset()
		self.right_encoder.reset()

	def set_motors(self, left_speed, right_speed):
		"""Sets motor speeds."""
		self.left_front.set(left_speed)
		self.left_rear.set(left_speed)
		self.right_front.set(right_speed)
		self.right_rear.set(right_speed)

	def start_travel(self, distance):
		print('start_travel')
		"""Resets encoders, stores the starting coordinates and heading, and sets the target travel distance."""
		self.reset_encoders()
		# Save the coordinate where travel starts.
		self.start_x = self.current_x
		self.start_y = self.current_y
		# Record the heading at the start of travel.
		self.travel_start_heading = self.getHeading()
		# Adjust target distance to account for overshoot.
		if distance >0:
			overshoot_distance=self.overshoot_distance
		else:
			overshoot_distance=-1*self.overshoot_distance
		self.target_distance = distance - overshoot_distance
		self.traveling = True

	def update_travel(self, base_speed=0.4):
		print('update_travel')

		if not self.traveling:
			return False

		# Determine travel direction and base speed.
		direction = 1 if self.target_distance > 0 else -1
		base_speed = base_speed * direction

		# Get the average distance traveled (in mm) from the encoders.
		left_dist = abs(self.left_encoder.getDistance())
		right_dist = abs(self.right_encoder.getDistance())
		avg_dist = (left_dist + right_dist) / 2.0

		# Compute encoder-based correction.
		encoder_diff = left_dist - right_dist
		encoder_correction = 0.0001 * encoder_diff  # Tune this value if needed

		# 🚀 FIXED: Desired heading should **always** be the original travel heading.
		desired_heading = self.travel_start_heading  # Do NOT flip by 180 degrees!

		# Get current heading and compute compass error
		current_heading = self.getHeading()
		compass_error = (desired_heading - current_heading + 180) % 360 - 180

		# Tune this gain carefully (reduce if oscillating)
		kp_compass = 0.005  # Try lowering to 0.005 or even 0.0025
		compass_correction = kp_compass * compass_error  # No need to flip correction

		# If the compass error is too large, override the encoder correction.
		threshold = 2  # degrees; adjust as needed.
		if abs(compass_error) > threshold:
			encoder_correction = 0
		else:
			compass_correction = 0

		# Compute motor speeds with corrections
		left_speed = base_speed - encoder_correction + compass_correction
		right_speed = base_speed + encoder_correction - compass_correction

		# Update the current coordinates using the travel distance.
		# (We use the original travel start heading to update coordinates.)
		heading_rad = math.radians(self.travel_start_heading)
		self.current_x = self.start_x + avg_dist * math.cos(heading_rad) * direction
		self.current_y = self.start_y + avg_dist * math.sin(heading_rad) * direction

		# Continue moving if the target distance hasn't been reached.
		if avg_dist < abs(self.target_distance):
			self.set_motors(left_speed, right_speed)
			return True
		else:
			self.set_motors(0, 0)
			self.traveling = False
			return False






	def start_turn(self, degrees, base_speed=0.4):
		print('start_turn')
		"""Initiates a turn by setting a target heading."""
		self.turning = True
		self.turn_target = self.getHeading() + degrees
		while self.turn_target > 360:
			self.turn_target -= 360
		while self.turn_target < 0:
			self.turn_target += 360
		self.turn_base_speed = base_speed if degrees > 0 else -base_speed

	def update_turn(self):
		if not self.turning:
			return False

		current_heading = self.getHeading()
		# Compute error as (target - current_heading)
		error = (self.turn_target - current_heading + 180) % 360 - 180
		# Normalize error to the range [-180, 180]
		while error > 180:
			error -= 360
		while error < -180:
			error += 360

		# If within tolerance, finish turning.
		if abs(error) < 2:
			self.set_motors(0, 0)
			self.turning = False
			return False

		# Compute the derivative term.
		# We'll store the previous error in self.prev_turn_error.
		if not hasattr(self, 'prev_turn_error'):
			self.prev_turn_error = error
		derivative = error - self.prev_turn_error
		self.prev_turn_error = error

		# PD controller gains. Tune these experimentally.
		kp_turn = 0.015  # Proportional gain
		kd_turn = 0.005  # Derivative gain

		# Calculate the turn speed command.
		turn_speed = kp_turn * error + kd_turn * derivative

		deadband=0.08
		if abs(turn_speed) < deadband:
			turn_speed=deadband if turn_speed >0 else -deadband

		# Clamp the command to the maximum speed allowed.
		max_speed = abs(self.turn_base_speed)
		if turn_speed > max_speed:
			turn_speed = max_speed
		elif turn_speed < -max_speed:
			turn_speed = -max_speed

		# Send commands to the motors. The signs here assume that a positive
		# turn_speed should turn the robot in the correct direction.
		print(turn_speed, error, self.turn_target, current_heading)
		self.set_motors(-turn_speed, turn_speed)
		return True


	def update_navigated_travel(self, base_speed=0.4):
		print('update_navigated_travel')

		if not self.navigating:
			return False

		# Compute vector toward the target
		dx = self.navigation_target_x - self.current_x
		dy = self.navigation_target_y - self.current_y
		target_distance = math.sqrt(dx**2 + dy**2)

		# 🚀 **Fix: Stop when close enough**
		target_tolerance = 50  # Stop within 50mm of the target
		if target_distance < target_tolerance:
			self.set_motors(0, 0)
			self.navigating = False
			return False

		# Compute desired heading
		desired_heading = math.degrees(math.atan2(dy, dx))
		if desired_heading < 0:
			desired_heading += 360  # Normalize

		# Get current heading and compute heading error
		current_heading = self.getHeading()
		heading_error = (desired_heading - current_heading + 180) % 360 - 180  # Normalize

		# Use small turn corrections while driving
		kp_turn = 0.01  # Adjust for smooth turning
		turn_correction = kp_turn * heading_error

		# 🚀 **Fix: Correct encoder distance calculation**
		left_distance = self.left_encoder.getDistance()
		right_distance = self.right_encoder.getDistance()
		avg_distance = (left_distance + right_distance) / 2.0  # Average of both encoders

		# Compute motor speeds (encoder straight + heading correction)
		left_speed = base_speed - turn_correction
		right_speed = base_speed + turn_correction

		# 🚀 Fix motor deadband (minimum speed)
		deadband = 0.2
		if abs(left_speed) < deadband:
			left_speed = deadband if left_speed > 0 else -deadband
		if abs(right_speed) < deadband:
			right_speed = deadband if right_speed > 0 else -deadband

		# **🚀 Fix: Update estimated position using encoders**
		heading_rad = math.radians(current_heading)
		self.current_x += avg_distance * math.cos(heading_rad)
		self.current_y += avg_distance * math.sin(heading_rad)

		# Move toward target
		self.set_motors(left_speed, right_speed)
		return True



	def start_navigation(self, current_x, current_y, target_x, target_y):
		print('start_navigation')

		# Save target coordinates
		self.navigation_target_x = target_x
		self.navigation_target_y = target_y

		# Reset encoders before travel to track correct distance
		self.reset_encoders()

		# Compute required turn to face the target
		dx = target_x - current_x
		dy = target_y - current_y
		target_distance = math.sqrt(dx**2 + dy**2)

		# Compute desired heading
		desired_heading = math.degrees(math.atan2(dy, dx))
		if desired_heading < 0:
			desired_heading += 360  # Normalize

		# Save desired heading
		self.navigation_heading = desired_heading

		# Start traveling immediately with heading correction
		self.start_travel(target_distance)
		self.navigating = True


	def update_navigation(self):

		if not self.navigating:
			return False

		# Update travel while dynamically adjusting heading when needed
		self.navigating = self.update_navigated_travel()
		return self.navigating



