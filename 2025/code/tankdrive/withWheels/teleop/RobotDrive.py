import wpilib
import ctre

class MyRobot(wpilib.TimedRobot):
	def robotInit(self) :
		self.DriveJoystick = wpilib.Joystick(0)
		
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX (2)

		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

	def autonomousInit(self):
		return False

	def autonomousPeriodic(self):
		return False

	def teleopInit(self):
		return False
	
	def teleopPeriodic(self):
		self.JoystickPeriodic()
		self.DrivePeriodic()
		return False
	
	
	def JoystickPeriodic (self):
		self.DRIVE_LEFT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(1)
		self.DRIVE_RIGHT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(5)

	def DrivePeriodic(self):
		self.LeftFrontMotor.set(-self.DRIVE_LEFT_THUMB_UPDOWN)
		self.LeftRearMotor.set(-self.DRIVE_LEFT_THUMB_UPDOWN)
		self.RightFrontMotor.set(self.DRIVE_RIGHT_THUMB_UPDOWN)
		self.RightRearMotor.set(self.DRIVE_RIGHT_THUMB_UPDOWN)		

if __name__ == "__main__" :
	wpilib.run(MyRobot)