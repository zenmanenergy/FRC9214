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

		self.IntakeMotor = rev.CANSparkMax(7,rev.CANSparkMax.MotorType.kBrushless)
		self.IntakeMotor.setSmartCurrentLimit(19) # set the maximum current limit to 20A
		self.IntakeMotor.setSecondaryCurrentLimit(25) # set the maximum current limit during current limit duration to 25A
        
		self.MaxDriveSpeed=0.5
		self.MaxArmSpeed=0.5
		self.MaxIntakeSpeed=0.6

	def autonomousInit(self):
		return False
	
	def autonomousPeriodic(self):
		return False
	
	def teleopInit(self):
		return False
	
	def teleopPeriodic(self):
		self.JoystickPeriodic()
		self.DrivePeriodic()
		self.ArmPeriodic()
		self.IntakePeriodic()
	
	def JoystickPeriodic (self):
		self.DRIVE_LEFT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(1)
		self.DRIVE_RIGHT_THUMB_UPDOWN = self.DriveJoystick.getRawAxis(5)

		self.ARM_LEFT_THUMB_UPDOWN = self.ArmJoystick.getRawAxis(1)
		self.ARM_RIGHT_THUMB_UPDOWN = self.ArmJoystick.getRawAxis(5)
	
	
	def DrivePeriodic(self):
		self.LeftFrontMotor.set(-self.DRIVE_LEFT_THUMB_UPDOWN*self.MaxDriveSpeed)
		self.LeftRearMotor.set(-self.DRIVE_LEFT_THUMB_UPDOWN*self.MaxDriveSpeed)
		self.RightFrontMotor.set(self.DRIVE_RIGHT_THUMB_UPDOWN*self.MaxDriveSpeed)
		self.RightRearMotor.set(self.DRIVE_RIGHT_THUMB_UPDOWN*self.MaxDriveSpeed)

	def ArmPeriodic(self):
		if self.ARM_LEFT_THUMB_UPDOWN > 0.05 or self.ARM_LEFT_THUMB_UPDOWN < -0.05:
			self.JackShaftMotor.set(-1*self.MaxArmSpeed*self.ARM_LEFT_THUMB_UPDOWN)
		else:
			self.JackShaftMotor.set(0)

	def IntakePeriodic(self):
		if self.ARM_RIGHT_THUMB_UPDOWN>0.05 or self.ARM_RIGHT_THUMB_UPDOWN<-0.05:
			self.IntakeMotor.set(-1*self.MaxIntakeSpeed*self.ARM_RIGHT_THUMB_UPDOWN)
		else:
			self.IntakeMotor.set(0)

if __name__=="__main__":
	wpilib.run(MyRobot)