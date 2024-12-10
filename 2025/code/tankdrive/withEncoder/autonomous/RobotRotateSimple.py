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
		self.ROBOT_WIDTH_MM = 600  # Distance between wheels, adjust as needed
		self.ROBOT_CIRCUMFERENCE_MM = self.ROBOT_WIDTH_MM * 3.141592653589793

		# Initialize encoder: Blue (Signal B) in DIO 1, Yellow (Signal A) in DIO 2
		self.left_encoder = wpilib.Encoder(1, 2)  # Left encoder on DIO 0, 1
		self.right_encoder = wpilib.Encoder(3, 4)  # Right encoder on DIO 2, 3

		# Invert the left encoder values to match motor configuration
		self.left_encoder.setReverseDirection(True)

		# Set distance per pulse
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
		self.DrivePeriodic()

		# Periodic encoder update
		self.logEncoderDistance()

	def JoystickPeriodic(self):
		self.DRIVE_LEFT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(1)
		self.DRIVE_RIGHT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(5)

	def DrivePeriodic(self):
		self.LeftFrontMotor.set(-self.DRIVE_LEFT_THUMB_UPDOWN)
		self.LeftRearMotor.set(-self.DRIVE_LEFT_THUMB_UPDOWN)
		self.RightFrontMotor.set(self.DRIVE_RIGHT_THUMB_UPDOWN)
		self.RightRearMotor.set(self.DRIVE_RIGHT_THUMB_UPDOWN)

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

if __name__ == "__main__":
	wpilib.run(MyRobot)
