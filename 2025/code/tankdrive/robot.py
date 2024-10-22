import wpilib # first robotics library
import ctre # Zippy wheel motor controller library
import rev # Zippy arm motor controller library

class MyRobot(wpilib.TimedRobot):
	def robotInit(self) : #basically defines the controller and motors
		self.DriveJoystick = wpilib.Joystick(0) #line 7 on the controller
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)

		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		self.ArmJoystick = wpilib.Joystick(1)
		self.JackShaftMotor = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
		self.JackShaftMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
		self.JackShaftMotor.setInverted(True)

		self.IntakeMotor = rev.CANSparkMax(7,rev.CANSparkMax.MotorType.kBrushless)
		self.IntakeMotor.setSecondaryCurrentLimit(25)
		self.IntakeMotor.setSmartCurrentLimit(19)

		self.ArmSpeed = 0.5
		self.IntakeSpeed = 0.6

		


	def autonomousInit(self) -> None:
		return super().autonomousInit()
	
	def autonomousPeriodic(self) -> None:
		return super().autonomousPeriodic()
	
	def teleopInit(self) -> None:
		return super().teleopInit()
	
	def teleopPeriodic(self) -> None:
		self.JoystickPeriodic()
		self.DrivePeriodic()
		self.ArmPeriodic()
		self.IntakePeriodic()
	


		return super().teleopPeriodic()
	
	def JoystickPeriodic (self):
		self.DRIVE_LEFT_THUMB_LEFTRIGHT = self.DriveJoystick.getRawAxis(0)
		self.DRIVE_LEFT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(1)
		self.DRIVE_RIGHT_THUMB_LEFTRIGHT = self.DriveJoystick.getRawAxis(4)
		self.DRIVE_RIGHT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(5)

		self.ARM_LEFT_THUMB_UPDOWN = self.ArmJoystick.getRawAxis(1)
		self.ARM_RIGHT_THUMB_UPDOWN = self.ArmJoystick.getRawAxis(5)
	
	def DrivePeriodic(self):
		self.DriveSpeed=1.0
		self.SetDrives(self.DriveSpeed*self.DRIVE_LEFT_THUMB_UPDOWN, self.DriveSpeed*self.DRIVE_RIGHT_THUMB_UPDOWN)
	
	def SetDrives(self, LeftSpeed: float, RightSpeed: float):
		self.LeftFrontMotor.set(-LeftSpeed)
		self.LeftRearMotor.set(-LeftSpeed)
		self.RightFrontMotor.set(RightSpeed)
		self.RightRearMotor.set(RightSpeed)

	def ArmPeriodic(self):
		if self.ARM_LEFT_THUMB_UPDOWN > 0.05:
			self.JackShaftMotor.set(-1 * self.ArmSpeed * self.ARM_LEFT_THUMB_UPDOWN)
		elif self.ARM_LEFT_THUMB_UPDOWN < -0.05:
			self.JackShaftMotor.set(-1 * self.ArmSpeed * self.ARM_LEFT_THUMB_UPDOWN)
		else:
			self.JackShaftMotor.set(0)
	def IntakePeriodic(self):
		if self.ARM_RIGHT_THUMB_UPDOWN>0.05:
			self.IntakeMotor.set(self.IntakeSpeed*self.ARM_RIGHT_THUMB_UPDOWN)
		elif self.ARM_RIGHT_THUMB_UPDOWN<-0.05:
			self.IntakeMotor.set(self.IntakeSpeed*self.ARM_RIGHT_THUMB_UPDOWN)
		else:
			self.IntakeMotor.set(0)		
if __name__=="__main__":
	wpilib.run(MyRobot)