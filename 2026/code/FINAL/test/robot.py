import wpilib # first robotics library

import rev # Zippy arm motor controller library

class MyRobot(wpilib.TimedRobot):
	def robotInit(self) : #basically defines the controller and motors




		self.elevator_motor = rev.SparkMax(10, rev.SparkMax.MotorType.kBrushless)
		self.shoulder_motor = rev.SparkMax(12, rev.SparkMax.MotorType.kBrushless)
		self.wrist_motor = rev.SparkMax(11, rev.SparkMax.MotorType.kBrushless)
		self.grabber_motor = rev.SparkMax(13, rev.SparkMax.MotorType.kBrushless)

		
		

		self.ArmJoystick = wpilib.Joystick(0)
		
		
		self.left_encoder = wpilib.Encoder(0, 1)
		self.right_encoder = wpilib.Encoder(8, 9)

	def autonomousInit(self):
		return False
	
	def autonomousPeriodic(self):
		return False
	
	def teleopInit(self):
		return False
	
	def teleopPeriodic(self):
		
		self.JoyStickPeriodic()
		self.elevator_motor.set(self.LeftThumbUPDOWN*.3)
		# self.shoulder_motor.set(shoulder_speed)
		# self.wrist_motor.set(wrist_speed)

	def JoyStickPeriodic(self):
		self.LeftThumbUPDOWN=self.ArmJoystick.getRawAxis(1)*-1  #reverse the direction, so up is positive
		if abs(self.LeftThumbUPDOWN) < 0.05:
			self.LeftThumbUPDOWN=0

		self.RightThumbUPDOWN=self.ArmJoystick.getRawAxis(5)*-1
		if abs(self.RightThumbUPDOWN) < 0.05:
			self.RightThumbUPDOWN=0
	
		self.RightThumbLEFTRIGHT=self.ArmJoystick.getRawAxis(4)
		if abs(self.RightThumbLEFTRIGHT) < 0.05:
			self.RightThumbLEFTRIGHT=0
		LeftTrigger=self.ArmJoystick.getRawAxis(2)*-1
		if abs(LeftTrigger) < 0.05:
			LeftTrigger=0
		RightTrigger=self.ArmJoystick.getRawAxis(3)
		if abs(RightTrigger) < 0.05:
			RightTrigger=0
		
		self.LeftORRightTrigger=0
		if LeftTrigger <0.1:
			self.LeftORRightTrigger=LeftTrigger
		if RightTrigger>0.1:
			self.LeftORRightTrigger=RightTrigger
		if abs(self.LeftORRightTrigger) < 0.05:
			self.LeftORRightTrigger=0


	
	
	
if __name__=="__main__":
	wpilib.run(MyRobot)