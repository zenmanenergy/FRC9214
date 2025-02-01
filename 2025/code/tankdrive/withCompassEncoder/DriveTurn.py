# CompassEncoder.py
import math
import wpilib
from navx import AHRS

class MotorGroup:
	"""
	A simple wrapper for a group of motor controllers.
	Each motor must implement a set(speed) method.
	"""
	def __init__(self, motors):
		self.motors = motors
	
	def set(self, speed):
		for motor in self.motors:
			motor.set(speed)

class DriveControl:
	"""
	DriveControl fuses encoder and compass (NavX) data to drive a specific distance
	in a straight line and to turn a specified number of degrees.
	"""
	def __init__(self, leftMotors, rightMotors,
		leftEncoderPorts, rightEncoderPorts,
		wheel_diameter_mm, encoder_cpr,
		navx_type=AHRS.NavXComType.kMXP_SPI, navx_update_rate=AHRS.NavXUpdateRate.k200Hz):
		# Save motor groups
		self.leftMotors = leftMotors
		self.rightMotors = rightMotors
		
		# Create and configure encoders
		# leftEncoderPorts and rightEncoderPorts should be 2-element tuples, e.g. (1,2) and (3,4)
		self.leftEncoder = wpilib.Encoder(leftEncoderPorts[0], leftEncoderPorts[1])
		self.rightEncoder = wpilib.Encoder(rightEncoderPorts[0], rightEncoderPorts[1])
		wheel_circumference = wheel_diameter_mm * math.pi
		distPerPulse = wheel_circumference / encoder_cpr
		self.leftEncoder.setDistancePerPulse(distPerPulse)
		self.rightEncoder.setDistancePerPulse(distPerPulse)
		
		# Create the NavX sensor
		self.navx = AHRS(navx_type, navx_update_rate)
		
		# State variables for driving straight
		self.drive_active = False
		self.drive_target_distance = 0	# in mm (absolute value)
		self.drive_direction = 1			# 1 for forward, -1 for backward
		self.drive_base_speed = 0
		self.drive_initial_heading = 0
		
		# State variables for turning
		self.turn_active = False
		self.turn_target = 0				# desired relative turn (degrees)
		self.turn_base_speed = 0
		self.turn_initial_heading = 0
		self.turn_desired_heading = 0

	def start_drive_distance(self, distance_mm, base_speed=0.4):
		"""
		Begin a drive-distance command.
		@param distance_mm: positive for forward, negative for backward.
		@param base_speed: base motor speed (0–1) before heading correction.
		"""
		self.drive_active = True
		self.drive_target_distance = abs(distance_mm)
		self.drive_direction = 1 if distance_mm > 0 else -1
		self.drive_base_speed = base_speed * self.drive_direction
		# Save the initial heading from the NavX to drive straight
		self.drive_initial_heading = self.navx.getYaw()
		# Reset encoders for an accurate distance reading
		self.leftEncoder.reset()
		self.rightEncoder.reset()
	
	def update_drive_distance(self):
		"""
		Call this repeatedly (for example, in teleopPeriodic) until it returns True.
		Returns True when the target distance has been reached.
		"""
		if not self.drive_active:
			return True
		
		# Compute the average distance traveled (in mm)
		left_distance = abs(self.leftEncoder.getDistance())
		right_distance = abs(self.rightEncoder.getDistance())
		avg_distance = (left_distance + right_distance) / 2.0
		
		# Compute heading error relative to the initial heading
		current_heading = self.navx.getYaw()
		heading_error = current_heading - self.drive_initial_heading
		while heading_error > 180:
			heading_error -= 360
		while heading_error < -180:
			heading_error += 360
		
		# Apply a simple proportional correction for heading error
		correction_gain = 0.01		# Tune this value as needed
		heading_correction = correction_gain * heading_error
		
		# Adjust left/right speeds based on the heading error
		left_speed = self.drive_base_speed - heading_correction
		right_speed = self.drive_base_speed + heading_correction
		
		self.leftMotors.set(left_speed)
		self.rightMotors.set(right_speed)
		
		# Check if we have reached the target distance (with a tolerance)
		if avg_distance >= self.drive_target_distance:
			self.leftMotors.set(0)
			self.rightMotors.set(0)
			self.drive_active = False
			return True
		return False
	
	def start_turn_degrees(self, angle_degrees, base_speed=0.3):
		"""
		Begin a turn command.
		@param angle_degrees: the relative angle (in degrees) to turn.
				Positive values turn right; negative values turn left.
		@param base_speed: maximum turning speed.
		"""
		self.turn_active = True
		self.turn_target = angle_degrees
		self.turn_base_speed = base_speed if angle_degrees > 0 else -base_speed
		self.turn_initial_heading = self.navx.getYaw()
		self.turn_desired_heading = self.turn_initial_heading + angle_degrees
		# Normalize desired heading to [-180, 180]
		while self.turn_desired_heading > 180:
			self.turn_desired_heading -= 360
		while self.turn_desired_heading < -180:
			self.turn_desired_heading += 360
	
	def update_turn_degrees(self):
		"""
		Call this repeatedly until the turn is complete.
		Returns True when the turn command is finished.
		"""
		if not self.turn_active:
			return True
		
		current_heading = self.navx.getYaw()
		error = self.turn_desired_heading - current_heading
		while error > 180:
			error -= 360
		while error < -180:
			error += 360
		
		# If the error is within tolerance, finish the turn
		if abs(error) < 2:		# tolerance in degrees
			self.leftMotors.set(0)
			self.rightMotors.set(0)
			self.turn_active = False
			return True
		
		# Use proportional control to determine turning speed
		kp_turn = 0.01		# Tune this gain as needed
		turn_speed = kp_turn * error
		# Clamp the turning speed to the magnitude of the base speed
		if turn_speed > abs(self.turn_base_speed):
			turn_speed = abs(self.turn_base_speed)
		elif turn_speed < -abs(self.turn_base_speed):
			turn_speed = -abs(self.turn_base_speed)
		
		# For turning in place, left motors run opposite to right motors
		self.leftMotors.set(-turn_speed)
		self.rightMotors.set(turn_speed)
		return False
