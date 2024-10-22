import wpilib # first robotics library
import ctre # Zippy wheel motor controller library
import rev # Zippy arm motor controller library

class MyRobot(wpilib.TimedRobot):
	def robotInit(self) : #basically defines the controller and motors
		self.DriveJoyStick = wpilib.Joystick(0) #line 7 on the controller
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)

		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		self.ArmJoyStick = wpilib.Joystick(1)
		self.JackShaftMotor = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
		self.JackShaftMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
		self.JackShaftMotor.setInverted(True)

	def autonomousInit(self) -> None:
		return super().autonomousInit()
	
	def autonomousPeriodic(self) -> None:
		return super().autonomousPeriodic()
	
	def teleopInit(self) -> None:
		return super().teleopInit()
	
	def teleopPeriodic(self) -> None:
		self.JoyStickPeriodic()
		self.DrivePeriodic()
		self.ArmPeriodic()


		return super().teleopPeriodic()
	
	def JoyStickPeriodic (self):
		self.DRIVE_LEFT_THUMB_LEFTRIGHT = self.DriveJoyStick.getRawAxis(0)
		self.DRIVE_LEFT_THUMB_UPDOWN = self.DriveJoyStick.getRawAxis(1)
		self.DRIVE_RIGHT_THUMB_LEFTRIGHT = self.DriveJoyStick.getRawAxis(4)
		self.DRIVE_RIGHT_THUMB_UPDOWN = self.DriveJoyStick.getRawAxis(5)

		self.ARM_RIGHT_TRIGGER = self.ArmJoyStick.getRawAxis(3)
		self.ARM_LEFT_TRIGGER = self.ArmJoyStick.getRawAxis(2)
	
	def DrivePeriodic(self):
		self.DriveSpeed=1.0
		self.SetDrives(self.DriveSpeed*self.DRIVE_LEFT_THUMB_UPDOWN, self.DriveSpeed*self.DRIVE_RIGHT_THUMB_UPDOWN)
	
	def SetDrives(self, LeftSpeed: float, RightSpeed: float):
		self.LeftFrontMotor.set(-LeftSpeed)
		self.LeftRearMotor.set(-LeftSpeed)
		self.RightFrontMotor.set(RightSpeed)
		self.RightRearMotor.set(RightSpeed)

	def ArmPeriodic(self):
		if self.ARM_RIGHT_TRIGGER > 0.05:
			self.JackShaftMotor.set(self.ARM_RIGHT_TRIGGER)
		elif self.ARM_LEFT_TRIGGER > 0.05:
			self.JackShaftMotor.set(-1*self.ARM_LEFT_TRIGGER)
		else:
			self.JackShaftMotor.set(0)

if __name__=="__main__":
	wpilib.run(MyRobot)