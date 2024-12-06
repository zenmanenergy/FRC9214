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
		Rotates the robot in place by the specified number of degrees using both encoders.
		"""
		print(f"Rotating to: {degrees:.2f} degrees")

		# Calculate the distance each wheel must travel to achieve the desired rotation
		rotation_distance_mm = (degrees / 360.0) * self.ROBOT_CIRCUMFERENCE_MM

		# Reset both encoders
		self.left_encoder.reset()
		self.right_encoder.reset()

		# Rotate the robot: left wheels backward, right wheels forward
		while abs(self.left_encoder.getDistance()) < rotation_distance_mm and abs(self.right_encoder.getDistance()) < rotation_distance_mm:
			self.LeftFrontMotor.set(-0.5)  # Adjust speed as necessary
			self.LeftRearMotor.set(-0.5)
			self.RightFrontMotor.set(0.5)
			self.RightRearMotor.set(0.5)

		# Stop the motors
		self.LeftFrontMotor.set(0)
		self.LeftRearMotor.set(0)
		self.RightFrontMotor.set(0)
		self.RightRearMotor.set(0)

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
