import math
import wpilib  # FIRST Robotics library
import ctre  # Zippy wheel motor controller library
import rev  # Zippy arm motor controller library

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):  # Initializes joystick, motors, and encoders
		# Joystick and motor setup
		self.DriveJoystick = wpilib.Joystick(0)  # Joystick port 0
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)
		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		# Encoder setup
		self.WHEEL_DIAMETER_MM = 152.4  # mm
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * math.pi
		self.ENCODER_CPR = 2048  # Counts per revolution for the encoder

		# Robot dimensions (wheelbase diameter for rotation calculation)
		self.ROBOT_WIDTH_MM = 508  # Distance between wheels, adjust as needed
		self.ROBOT_CIRCUMFERENCE_MM = self.ROBOT_WIDTH_MM * math.pi

		# Initialize left and right encoders
		self.left_encoder = wpilib.Encoder(0, 1)  # Left encoder on DIO 0, 1
		self.right_encoder = wpilib.Encoder(2, 3)  # Right encoder on DIO 2, 3
		self.left_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)
		self.right_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)

		# Rotation parameters
		self.max_speed = 0.5  # Maximum motor speed
		self.min_speed = 0.2  # Minimum motor speed for smooth startup/stop
		self.accel_distance = 50  # Distance (mm) over which to accelerate/decelerate


	def autonomousInit(self):
		return False
	
	def autonomousPeriodic(self):
		return False
	
	def teleopInit(self):
		# Reset encoder distances at the start of teleop
		self.left_encoder.reset()
		self.right_encoder.reset()

	def teleopPeriodic(self):
		# Periodic joystick and driving updates
		self.JoystickPeriodic()

		# Call rotation functions based on button presses
		self.checkForRotation()

	def JoystickPeriodic(self):
		self.DRIVE_BUTTON_A = self.DriveJoystick.getRawButton(1)  # A button
		self.DRIVE_BUTTON_B = self.DriveJoystick.getRawButton(2)  # B button
		self.DRIVE_BUTTON_X = self.DriveJoystick.getRawButton(3)  # X button
		self.DRIVE_BUTTON_Y = self.DriveJoystick.getRawButton(4)  # Y button

	def rotateRobot(self, degrees):
		"""
		Rotates the robot in place by the specified number of degrees using both encoders,
		with acceleration and deceleration.
		"""
		print(f"Rotating to: {degrees:.2f} degrees")

		# Calculate the distance each wheel must travel to achieve the desired rotation
		rotation_distance_mm = (degrees / 360.0) * self.ROBOT_CIRCUMFERENCE_MM


		
		while True:
			# Get the distances traveled by each encoder
			left_distance = abs(self.left_encoder.getDistance())
			right_distance = abs(self.right_encoder.getDistance())

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
			self.LeftFrontMotor.set(-speed)  # Left wheels backward
			self.LeftRearMotor.set(-speed)
			self.RightFrontMotor.set(speed)  # Right wheels forward
			self.RightRearMotor.set(speed)

		# Stop the motors after the rotation
		self.LeftFrontMotor.set(0)
		self.LeftRearMotor.set(0)
		self.RightFrontMotor.set(0)
		self.RightRearMotor.set(0)

		print("Rotation complete.")


	def checkForRotation(self):
		"""
		Checks for button presses and calls rotateRobot() with the corresponding angle.
		"""
		if self.DRIVE_BUTTON_Y:
			self.rotateRobot(0)  # Y button = 0 degrees
		elif self.DRIVE_BUTTON_X:
			self.rotateRobot(90)  # X button = 90 degrees
		elif self.DRIVE_BUTTON_A:
			self.rotateRobot(180)  # A button = 180 degrees
		elif self.DRIVE_BUTTON_B:
			self.rotateRobot(270)  # B button = 270 degrees


if __name__ == "__main__":
	wpilib.run(MyRobot)
