import wpilib  # FIRST Robotics library
import ctre  # Motor controller library
import rev  # Additional motor controller library

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):  # Initializes motors and encoder
		# Joystick and motor setup (joystick not used in autonomous)
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)
		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		# Encoder setup
		self.WHEEL_DIAMETER_MM = 152.4  # mm
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.141592653589793
		self.ENCODER_CPR = 2048  # Counts per revolution for the encoder

		# Initialize encoder: Blue (Signal B) in DIO 1, Yellow (Signal A) in DIO 2
		self.left_encoder = wpilib.Encoder(0, 1)  # Left encoder on DIO 0, 1
		self.right_encoder = wpilib.Encoder(2, 3)  # Right encoder on DIO 2, 3
		self.left_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)
		self.right_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)

		# Travel parameters
		self.max_speed = 0.5  # Maximum motor speed
		self.min_speed = 0.2  # Minimum motor speed for smooth startup/stop
		self.accel_distance = 100  # Distance (mm) over which to accelerate/decelerate


	def autonomousInit(self):
		# Reset the encoder at the start of autonomous
		self.encoder.reset()

		# Flag to indicate the current step of the autonomous sequence
		self.step = 0

	def autonomousPeriodic(self):
		# Step 0: Move forward 2 meters
		if self.step == 0:
			if not self.travelDistance(2000):  # 2000 mm = 2 meters
				self.step += 1  # Move to the next step

		# Step 1: Move backward 2 meters
		elif self.step == 1:
			if not self.travelDistance(-2000):  # -2000 mm = -2 meters
				self.step += 1  # End the sequence

	def travelDistance(self, distance_mm):
		"""
		Makes the robot travel a specified distance in millimeters at a controlled speed.
		Uses two encoders for accurate distance tracking and applies acceleration/deceleration.
		Returns True if the robot is still moving, False if it has reached the target distance.
		"""
		# Determine the direction of travel
		direction = 1 if distance_mm > 0 else -1
		target_distance = abs(distance_mm)

		
		# Calculate the average distance traveled by both encoders
		left_distance = abs(self.left_encoder.getDistance())
		right_distance = abs(self.right_encoder.getDistance())
		average_distance = (left_distance + right_distance) / 2

		# Print encoder values for debugging
		print(f"Left Distance: {left_distance:.2f} mm, Right Distance: {right_distance:.2f} mm, Average: {average_distance:.2f} mm")

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
			self.LeftFrontMotor.set(direction * speed)
			self.LeftRearMotor.set(direction * speed)
			self.RightFrontMotor.set(-1 * direction * speed)
			self.RightRearMotor.set(-1 * direction * speed)
			return True
		else:
			# Stop the motors
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			return False



if __name__ == "__main__":
	wpilib.run(MyRobot)
