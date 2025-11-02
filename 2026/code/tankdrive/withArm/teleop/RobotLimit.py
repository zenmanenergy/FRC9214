import wpilib
import rev

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.ArmJoystick = wpilib.Joystick(1)
		self.JackShaftMotor = rev.CANSparkMax(6,rev.CANSparkMax.MotorType.KBrushless)
		self.JackShaftMotor .setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
		self.limit_switch = wpilib.DigitalInput(0)
		self.ArmSpeed = 0.5

		self.limit_switch = wpilib.DigitalInput(0)
		self.JackShaftEncoder = self.JackShaftMotor.getEncoder()



		self.retracting = False


	def autonomousInit(self):
		return False

	def autonomousPeriodic(self):
		return False


	def teleopInit(self):
		

		return False
	




	
	def teleopPeriodic(self):
		#Check if limit switch is on a DIO port (e.g., DIO0)
		if self.limit_switch.get() == False:  # Assuming False means pressed
			print("Limit switch is pressed.")
		else:
			print("Limit switch is not pressed.") 

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
		self.LeftFrontMotor.set(-self.DRIVE_LEFT_THUMB_UPDOWN)
		self.LeftRearMotor.set(-self.DRIVE_LEFT_THUMB_UPDOWN)
		self.RightFrontMotor.set(self.DRIVE_RIGHT_THUMB_UPDOWN)
		self.RightRearMotor.set(self.DRIVE_RIGHT_THUMB_UPDOWN)

	def ArmPeriodic(self):
		# Get the current encoder position of the JackShaftMotor
		current_position = self.JackShaftEncoder.getPosition()
	
if __name__ == "__main__":
	wpilib.run(MyRobot)
