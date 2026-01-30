import math

class PIDController:
	"""Simple PID controller for autonomous driving."""
	def __init__(self, kp=0.0, ki=0.0, kd=0.0, name="PID"):
		self.kp = kp  # Proportional gain
		self.ki = ki  # Integral gain
		self.kd = kd  # Derivative gain
		self.name = name  # Name for debugging
		
		self.prev_error = 0.0
		self.integral = 0.0
		
		# Debug tracking
		self.last_p_term = 0.0
		self.last_i_term = 0.0
		self.last_d_term = 0.0
		self.last_output = 0.0
		self.last_error = 0.0
	
	def calculate(self, error):
		"""Calculate PID output given an error value."""
		# Proportional term
		self.last_p_term = self.kp * error
		
		# Integral term (accumulate error)
		self.integral += error
		self.last_i_term = self.ki * self.integral
		
		# Derivative term (rate of change)
		derivative = error - self.prev_error
		self.last_d_term = self.kd * derivative
		
		# Store for next iteration
		self.prev_error = error
		self.last_error = error
		
		# Return sum of all terms
		self.last_output = self.last_p_term + self.last_i_term + self.last_d_term
		return self.last_output
	
	def reset(self):
		"""Reset PID state for new control sequence."""
		self.prev_error = 0.0
		self.integral = 0.0
		self.last_p_term = 0.0
		self.last_i_term = 0.0
		self.last_d_term = 0.0
		self.last_output = 0.0
		self.last_error = 0.0
	
	def print_debug(self):
		"""Print PID debug information."""
		print(f"[{self.name}] Error: {self.last_error:.2f} | P: {self.last_p_term:.3f} | I: {self.last_i_term:.3f} | D: {self.last_d_term:.3f} | Output: {self.last_output:.3f}")

class Drive:
	
	# Stores many controller functions and joystick control functions in one instance
	def __init__(self, DriveJoystick, left_front, left_rear, right_front, right_rear, odometry=None):
		self.DriveJoystick = DriveJoystick
		self.left_front = left_front
		self.left_rear = left_rear
		self.right_front = right_front
		self.right_rear = right_rear
		self.odometry = odometry

		# Sets initial position of thumb sticks at 0
		self.DRIVE_LEFT_THUMB_UPDOWN = 0
		self.DRIVE_RIGHT_THUMB_UPDOWN = 0

		# These two variables set the max speed power for either side of motors
		self.leftSpeedFactor = 0.5
		self.rightSpeedFactor = 0.5
		
		# Auto-drive tracking
		self.driving_to_position = False
		self.target_x = 0.0
		self.target_y = 0.0
		self.target_heading = 0.0  # Target heading in radians
		self.position_tolerance = 5.0  # cm tolerance
		self.heading_tolerance = 5.0  # degrees tolerance
		
		# Initialize PID controllers with names for debugging
		self.distance_pid = PIDController(kp=0.008, ki=0.0, kd=0.001, name="Distance")
		self.heading_pid = PIDController(kp=0.5, ki=0.01, kd=0.1, name="Heading")

	def reset(self):
		self.set_motors(0, 0)

	def JoyStickPeriodic(self):
		# This determines what position either thumb stick is at
		left_value = -1 * self.DriveJoystick.getRawAxis(1)
		right_value = -1 * self.DriveJoystick.getRawAxis(5)

		if abs(left_value) > 0.05:  # this checks if the joystick is in a neutral position
			self.DRIVE_LEFT_THUMB_UPDOWN = left_value * self.leftSpeedFactor
		else:
			self.DRIVE_LEFT_THUMB_UPDOWN = 0  # If neutral, no output, robot stops

		if abs(right_value) > 0.05:  # this checks if the joystick is in a neutral position
			self.DRIVE_RIGHT_THUMB_UPDOWN = right_value * self.rightSpeedFactor
		else:
			self.DRIVE_RIGHT_THUMB_UPDOWN = 0  # If neutral, no output

	# Creates a function to assign thumb sticks to drive functions, robot stops
	def periodic(self):
		# If auto-driving to position, do that instead of joystick control
		if self.driving_to_position and self.odometry:
			self.auto_drive_to_position()
		else:
			self.JoyStickPeriodic()
			
		self.set_motors(self.DRIVE_LEFT_THUMB_UPDOWN, self.DRIVE_RIGHT_THUMB_UPDOWN)
		
	# Creates a function to assign stored speed values to the motors
	def set_motors(self, left_speed, right_speed):
		self.left_front.set(left_speed)
		self.left_rear.set(left_speed)
		self.right_front.set(right_speed)
		self.right_rear.set(right_speed)
	
	def drive_to_position(self, x, y, heading=None):
		"""Start autonomous drive to target position (x, y) in centimeters.
		
		Args:
			x, y: Target position in centimeters
			heading: Target heading in radians (optional). If None, robot maintains current heading.
		"""
		if self.odometry:
			self.target_x = x
			self.target_y = y
			if heading is not None:
				self.target_heading = heading
			else:
				# If no heading specified, keep current heading
				self.target_heading = self.odometry.get_heading()
			self.driving_to_position = True
			# Reset PID controllers for clean start
			self.distance_pid.reset()
			self.heading_pid.reset()
			print(f"Starting auto-drive to ({x:.2f}, {y:.2f}) cm, heading: {math.degrees(self.target_heading):.2f}°")
	
	def auto_drive_to_position(self):
		"""Autonomous driving using PID control for smooth motion and heading."""
		if not self.odometry:
			return
		
		# Get current position and heading
		current_x, current_y = self.odometry.get_position()
		current_heading = self.odometry.get_heading()
		
		# Calculate distance and angle to target position
		dx = self.target_x - current_x
		dy = self.target_y - current_y
		distance_to_target = math.sqrt(dx**2 + dy**2)
		
		# Desired heading to target position
		desired_heading_to_target = math.atan2(dy, dx)
		
		# Check if we've reached the target position
		position_reached = distance_to_target < self.position_tolerance
		
		# Calculate heading error to target heading (for final orientation)
		heading_error_to_target = self.target_heading - current_heading
		heading_error_to_target = math.atan2(math.sin(heading_error_to_target), math.cos(heading_error_to_target))
		heading_error_magnitude = abs(heading_error_to_target)
		
		# Check if heading is reached
		heading_reached = math.degrees(heading_error_magnitude) < self.heading_tolerance
		
		# If both position and heading reached, stop
		if position_reached and heading_reached:
			self.DRIVE_LEFT_THUMB_UPDOWN = 0
			self.DRIVE_RIGHT_THUMB_UPDOWN = 0
			self.driving_to_position = False
			print(f"Reached target position ({self.target_x:.2f}, {self.target_y:.2f}) and heading {math.degrees(self.target_heading):.2f}°")
			return
		
		# If position reached, focus on rotating to target heading
		if position_reached:
			# Stop forward motion and focus on heading
			forward_speed = 0.0
			turn_speed = self.heading_pid.calculate(heading_error_to_target)
			turn_speed = max(-0.5, min(0.5, turn_speed))  # Clamp turn speed
		else:
			# Drive toward target position while also rotating toward final heading
			# Calculate heading error to target position
		heading_error_to_position = desired_heading_to_target - current_heading
		heading_error_to_position = math.atan2(math.sin(heading_error_to_position), math.cos(heading_error_to_position))
		
		# PID outputs
		forward_speed = self.distance_pid.calculate(distance_to_target)
		
		# Exponential speed curve - slows down aggressively as it approaches target
		speed_multiplier = (distance_to_target / 100.0) ** 3.0
		speed_multiplier = max(0.1, min(1.0, speed_multiplier))  # Clamp between 0.1 and 1.0
		
		forward_speed = forward_speed * speed_multiplier
		forward_speed = max(0.25, min(0.5, forward_speed))  # Allow full range from 0.25 to 0.5
		
		turn_speed = self.heading_pid.calculate(heading_error_to_position)
		turn_speed = max(-0.5, min(0.5, turn_speed))  # Clamp turn speed
		
		# Add braking effect when very close to target
		if distance_to_target < 14:
			forward_speed = -0.05  # Reverse at low speed to brake
		
		# Debug output (every few loops for readability)
		if int(distance_to_target) % 5 == 0:
			print(f"\n--- Auto-Drive Debug ---")
			print(f"Distance to target: {distance_to_target:.2f} cm")
			print(f"Heading to position: {math.degrees(desired_heading_to_target):.2f}°, Error: {math.degrees(heading_error_to_position):.2f}°")
			print(f"Target heading: {math.degrees(self.target_heading):.2f}°, Error: {math.degrees(heading_error_to_target):.2f}°")
			self.distance_pid.print_debug()
			self.heading_pid.print_debug()
			print(f"Motor speeds - Left: {self.DRIVE_LEFT_THUMB_UPDOWN:.3f}, Right: {self.DRIVE_RIGHT_THUMB_UPDOWN:.3f}")
			return
	
	# Tank drive mixing: differential drive
		self.DRIVE_LEFT_THUMB_UPDOWN = forward_speed - turn_speed
		self.DRIVE_RIGHT_THUMB_UPDOWN = forward_speed + turn_speed
		
		# Clamp motor speeds to [-1, 1]
		self.DRIVE_LEFT_THUMB_UPDOWN = max(-1.0, min(1.0, self.DRIVE_LEFT_THUMB_UPDOWN))
		self.DRIVE_RIGHT_THUMB_UPDOWN = max(-1.0, min(1.0, self.DRIVE_RIGHT_THUMB_UPDOWN))