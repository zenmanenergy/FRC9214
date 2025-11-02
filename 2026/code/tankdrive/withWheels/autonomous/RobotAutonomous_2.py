"""
Autonomous 2:
This program defines a more advanced FRC robot autonomous sequence using WPILib and CTRE libraries.
The robot executes a series of timed drive actions during the autonomous phase. The sequence includes
driving forward and backward at specified speeds and durations. A step counter tracks the progress
of the autonomous routine.
"""

import wpilib
from phoenix5 import WPI_TalonSRX
class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.LeftFrontMotor = WPI_TalonSRX(1)
		self.LeftRearMotor = WPI_TalonSRX(2)
		self.RightFrontMotor = WPI_TalonSRX(3)
		self.RightRearMotor = WPI_TalonSRX(4)
		self.timer = wpilib.Timer()
		self.driveStep = 0

	def autonomousInit(self):
		self.driveStep = 0

	def autonomousPeriodic(self):
		if self.driveStep == 0:
			if self.drive_forward_for_time(-0.4, 0.75):
				self.driveStep += 1

		elif self.driveStep == 1:
			if self.drive_forward_for_time(0.4, 0.75):
				self.driveStep += 1

		elif self.driveStep == 2:
			if self.drive_forward_for_time(-0.05, 0.75):
				self.driveStep += 1

	def teleopInit(self):
		pass
	
	def teleopPeriodic(self):
		pass

	def drive_forward_for_time(self, speed, duration):
		if self.timer.get() == 0:
			self.timer.start()
			self.LeftFrontMotor.set(-speed)
			self.LeftRearMotor.set(-speed)
			self.RightFrontMotor.set(speed)
			self.RightRearMotor.set(speed)

		if self.timer.get() >= duration:
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			self.timer.stop()
			self.timer.reset()
			return True

		return False

if __name__ == "__main__":
	wpilib.run(MyRobot)
