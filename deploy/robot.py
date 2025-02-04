import wpilib # first robotics library

import rev # Zippy arm motor controller library

class MyRobot(wpilib.TimedRobot):
	def robotInit(self) : #basically defines the controller and motors

		self.joystick = wpilib.Joystick(1)
		self.JackShaftMotor = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
		# self.JackShaftMotor.setIdleMode(rev.SparkMax.IdleMode.kBrake)

		self.ArmSpeed=0.2

	def autonomousInit(self):
		return False
	
	def autonomousPeriodic(self):
		return False
	
	def teleopInit(self):
		return False
	
	def teleopPeriodic(self):
		
		if self.joystick.getRawButtonReleased(4):  # Y button to move forward 1000 mm
			print("Y button")




	
	def JoystickPeriodic (self):
		
        
		self.ARM_LEFT_THUMB_UPDOWN = self.ArmJoystick.getRawButtonReleased(1)
		
	
	
	
    
	def ArmPeriodic(self):
		if self.ARM_LEFT_THUMB_UPDOWN > 0.05 or self.ARM_LEFT_THUMB_UPDOWN < -0.05:
			print("turn on motor")
			self.JackShaftMotor.set(-1*self.ArmSpeed*self.ARM_LEFT_THUMB_UPDOWN)
		else:
			print("turn off motor")
			self.JackShaftMotor.set(0)

if __name__=="__main__":
	wpilib.run(MyRobot)