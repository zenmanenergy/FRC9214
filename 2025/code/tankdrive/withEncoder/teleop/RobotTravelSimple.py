import wpilib  # FIRST Robotics library
import ctre  # Zippy wheel motor controller library
import rev  # Zippy arm motor controller library

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):  # Initializes joystick, motors, and encoder
		# Joystick and motor setup
		self.DriveJoystick = wpilib.Joystick(0)  # Joystick port 0
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)
		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		# Encoder setup
		self.WHEEL_DIAMETER_MM = 152.4  # mm
		self.WHEEL_CIRCUMFERENCE_MM = self.WHEEL_DIAMETER_MM * 3.141592653589793
		self.ENCODER_CPR = 2048  # Counts per revolution for the encoder

		# Robot dimensions (wheelbase diameter for rotation calculation)
		self.ROBOT_WIDTH_MM = 508  # Distance between wheels, adjust as needed
		self.ROBOT_CIRCUMFERENCE_MM = self.ROBOT_WIDTH_MM * 3.141592653589793

		# Initialize encoder: Blue (Signal B) in DIO 1, Yellow (Signal A) in DIO 2
		self.left_encoder = wpilib.Encoder(0, 1)  # Left encoder on DIO 0, 1
		self.right_encoder = wpilib.Encoder(2, 3)  # Right encoder on DIO 2, 3
		self.left_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)
		self.right_encoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)

	def autonomousInit(self):
		return False
	
	def autonomousPeriodic(self):
		return False
	
	def teleopInit(self):
		# Reset encoder distance at the start of teleop
		self.encoder.reset()

	def teleopPeriodic(self):
		# Periodic joystick and driving updates
		self.JoystickPeriodic()


		# Call rotation functions based on button presses
		self.checkForTravel()

	def JoystickPeriodic(self):
		self.DRIVE_BUTTON_A = self.DriveJoystick.getRawButton(1)  # A button
		self.DRIVE_BUTTON_Y = self.DriveJoystick.getRawButton(4)  # Y button

	def checkForTravel(self):
		"""
		Checks for button presses and calls rotateRobot() with the corresponding angle.
		"""
		if self.DRIVE_BUTTON_Y:
			self.travelDistance(1000)  # Y button = 0 degrees
		elif self.DRIVE_BUTTON_A:
			self.travelDistance(1000)  # A button = 180 degrees

	def travelDistance(self, distance_mm):
		"""
		Makes the robot travel a specified distance in millimeters at a constant speed.
		Uses two encoders for more accurate distance tracking.
		Returns True if the robot is still moving, False if it has reached the target distance.
		"""
		# Determine the direction of travel
		direction = 1 if distance_mm > 0 else -1
		target_distance = abs(distance_mm)

		# Travel speed
		speed = 0.5  # Adjust this speed as necessary

		# Calculate the average distance traveled by both encoders
		left_distance = abs(self.left_encoder.getDistance())
		right_distance = abs(self.right_encoder.getDistance())
		average_distance = (left_distance + right_distance) / 2

		# Check if the target distance is reached
		if average_distance < target_distance:
			# Apply constant speed to the motors
			self.LeftFrontMotor.set(direction * speed)
			self.LeftRearMotor.set(direction * speed)
			self.RightFrontMotor.set(direction * speed)
			self.RightRearMotor.set(direction * speed)
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
