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

		# Initialize encoder: Blue (Signal B) in DIO 1, Yellow (Signal A) in DIO 2
		self.LeftEncoder = wpilib.Encoder(1, 2)
		self.LeftEncoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)
		self.RightEncoder = wpilib.Encoder(3, 4)
		self.RightEncoder.setDistancePerPulse(self.WHEEL_CIRCUMFERENCE_MM / self.ENCODER_CPR)
	def autonomousInit(self):
		return False
	
	def autonomousPeriodic(self):
		return False
	
	def teleopInit(self):
		# Reset encoder distance at the start of teleop
		self.LeftEncoder.reset()
		self.RightEncoder.reset()
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

	def logEncoderDistance(self):
		# Retrieve and print the distance traveled
		LeftDistance_traveled_mm = -1*self.LeftEncoder.getDistance()
		RightDistance_traveled_mm = 1*self.RightEncoder.getDistance()
		print(f"Distance traveled: {LeftDistance_traveled_mm:.2f} mm {RightDistance_traveled_mm:.2f} mm")


if __name__ == "__main__":
	wpilib.run(MyRobot)
