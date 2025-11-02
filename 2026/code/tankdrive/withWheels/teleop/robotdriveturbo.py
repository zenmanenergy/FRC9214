import wpilib
from phoenix5 import WPI_TalonSRX

class MyRobot(wpilib.TimedRobot):
	def robotInit(self) :
		self.DriveJoystick = wpilib.Joystick(0)
		
		self.LeftFrontMotor = WPI_TalonSRX(1)
		self.LeftRearMotor = WPI_TalonSRX (4)

		self.RightFrontMotor = WPI_TalonSRX(3)
		self.RightRearMotor = WPI_TalonSRX(2)

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
		self.TURBO = self.DriveJoystick.getRawButton(5)

	def DrivePeriodic(self):
		self.Turbo()
		self.LeftFrontMotor.set(-self.DRIVE_LEFT_THUMB_UPDOWN * self.SPEED)
		self.LeftRearMotor.set(-self.DRIVE_LEFT_THUMB_UPDOWN * self.SPEED)
		self.RightFrontMotor.set(self.DRIVE_RIGHT_THUMB_UPDOWN * self.SPEED)
		self.RightRearMotor.set(self.DRIVE_RIGHT_THUMB_UPDOWN * self.SPEED)		

	def Turbo(self):
		print(self.TURBO)
		if self.TURBO:
			self.SPEED = 1
		else:
			self.SPEED = 0.5

if __name__ == "__main__" :
	wpilib.run(MyRobot)
