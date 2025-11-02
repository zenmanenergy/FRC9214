import math


class Movement:
	def __init__(self, robot, max_speed=0.5, min_speed=0.2, accel_distance=100):
		"""
		Initialize the movement library with the robot object and tuning parameters.
		"""
		self.robot = robot
		self.max_speed = max_speed
		self.min_speed = min_speed
		self.accel_distance = accel_distance
		self.x = 0.0  # Robot's X-coordinate in mm
		self.y = 0.0  # Robot's Y-coordinate in mm
		self.heading = 0.0  # Robot's heading in radians (0 = facing forward)

	def setCoordinate(self, x, y, heading_degrees=None):
		"""
		Sets the robot's current position and optionally its heading.
		"""
		self.x = x
		self.y = y
		if heading_degrees is not None:
			self.heading = math.radians(heading_degrees)
		print(f"Set coordinate to X: {x}, Y: {y}, Heading: {math.degrees(self.heading):.2f}°")

	def rotate(self, degrees):
		"""
		Rotates the robot in place by the specified number of degrees using both encoders.
		"""
		print(f"Rotating to: {degrees:.2f} degrees")

		# Calculate the distance each wheel must travel to achieve the desired rotation
		rotation_distance_mm = (degrees / 360.0) * self.robot.ROBOT_CIRCUMFERENCE_MM

		# Reset both encoders
		self.robot.left_encoder.reset()
		self.robot.right_encoder.reset()

		while True:
			# Get the distances traveled by each encoder
			left_distance = abs(self.robot.left_encoder.getDistance())
			right_distance = abs(self.robot.right_encoder.getDistance())

			# Calculate the average distance traveled
			average_distance = (left_distance + right_distance) / 2

			# Break the loop if the target rotation distance is reached
			if average_distance >= rotation_distance_mm:
				break

			# Calculate the remaining distance
			remaining_distance = rotation_distance_mm - average_distance

			# Adjust speed based on remaining distance (deceleration)
			if remaining_distance < self.accel_distance:
				speed = max(self.min_speed, self.max_speed * (remaining_distance / self.accel_distance))
			# Accelerate at the start
			elif average_distance < self.accel_distance:
				speed = max(self.min_speed, self.max_speed * (average_distance / self.accel_distance))
			# Maintain maximum speed in the middle
			else:
				speed = self.max_speed

			# Apply speed to the motors for rotation
			self.robot.LeftFrontMotor.set(-speed)  # Left wheels backward
			self.robot.LeftRearMotor.set(-speed)
			self.robot.RightFrontMotor.set(speed)  # Right wheels forward
			self.robot.RightRearMotor.set(speed)

		# Stop the motors after the rotation
		self.robot.LeftFrontMotor.set(0)
		self.robot.LeftRearMotor.set(0)
		self.robot.RightFrontMotor.set(0)
		self.robot.RightRearMotor.set(0)

		# Update heading
		self.heading += math.radians(degrees)
		self.heading %= 2 * math.pi  # Keep heading within [0, 2π)
		print(f"Rotation complete. New heading: {math.degrees(self.heading):.2f}°")

	def travelDistance(self, distance_mm):
		"""
		Makes the robot travel a specified distance in millimeters at a controlled speed.
		Uses two encoders for accurate distance tracking and applies acceleration/deceleration.
		"""
		# Determine the direction of travel
		direction = 1 if distance_mm > 0 else -1
		target_distance = abs(distance_mm)

		# Calculate the average distance traveled by both encoders
		left_distance = abs(self.robot.left_encoder.getDistance())
		right_distance = abs(self.robot.right_encoder.getDistance())
		average_distance = (left_distance + right_distance) / 2

		# Check if the target distance is reached
		if average_distance < target_distance:
			# Calculate the remaining distance
			remaining_distance = target_distance - average_distance

			# Adjust speed based on remaining distance (deceleration)
			if remaining_distance < self.accel_distance:
				speed = max(self.min_speed, self.max_speed * (remaining_distance / self.accel_distance))
			# Accelerate at the start
			elif average_distance < self.accel_distance:
				speed = max(self.min_speed, self.max_speed * (average_distance / self.accel_distance))
			# Maintain maximum speed in the middle
			else:
				speed = self.max_speed

			# Apply speed to the motors
			self.robot.LeftFrontMotor.set(direction * speed)
			self.robot.LeftRearMotor.set(direction * speed)
			self.robot.RightFrontMotor.set(-1 * direction * speed)
			self.robot.RightRearMotor.set(-1 * direction * speed)
			return True
		else:
			# Stop the motors
			self.robot.LeftFrontMotor.set(0)
			self.robot.LeftRearMotor.set(0)
			self.robot.RightFrontMotor.set(0)
			self.robot.RightRearMotor.set(0)

			# Update coordinates based on distance traveled
			distance_traveled = direction * target_distance
			self.x += distance_traveled * math.cos(self.heading)
			self.y += distance_traveled * math.sin(self.heading)
			print(f"Updated position: X: {self.x:.2f}, Y: {self.y:.2f}")
			return False
