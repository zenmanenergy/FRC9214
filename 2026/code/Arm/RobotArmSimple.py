import wpilib # first robotics library

import rev # Zippy arm motor controller library

class MyRobot(wpilib.TimedRobot):
	def robotInit(self) : #basically defines the controller and motors


		self.ArmJoystick = wpilib.Joystick(1)
		self.JackShaftMotor = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
		self.JackShaftMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

		self.ArmSpeed=0.5

	def autonomousInit(self):
		return False
	
	def autonomousPeriodic(self):
		return False
	
	def teleopInit(self):
		return False
	
	def teleopPeriodic(self):
		self.JoystickPeriodic()
		self.ArmPeriodic()


	
	def JoystickPeriodic (self):
		
        
		self.ARM_LEFT_THUMB_UPDOWN = self.ArmJoystick.getRawAxis(1)
		
	
	
	
    
	def ArmPeriodic(self):
		if self.ARM_LEFT_THUMB_UPDOWN > 0.05 or self.ARM_LEFT_THUMB_UPDOWN < -0.05:
			self.JackShaftMotor.set(-1*self.ArmSpeed*self.ARM_LEFT_THUMB_UPDOWN)
		else:
			self.JackShaftMotor.set(0)

if __name__=="__main__":
	wpilib.run(MyRobot)