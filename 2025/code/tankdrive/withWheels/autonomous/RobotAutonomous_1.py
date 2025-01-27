"""
Autonomous 1:
This program defines a basic FRC robot that uses WPILib and CTRE libraries to control four motors.
The robot is programmed to autonomously drive forward at a specified speed for a certain duration
during the autonomous phase. The drive function resets automatically for subsequent calls.
"""

import wpilib
from phoenix5 import WPI_TalonSRX


class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.LeftFrontMotor = WPI_TalonSRX(1)
		self.LeftRearMotor = WPI_TalonSRX (4)

		self.RightFrontMotor = WPI_TalonSRX(3)
		self.RightRearMotor = WPI_TalonSRX(2)

		self.timer = wpilib.Timer()

	def autonomousInit(self):
		self.timer.reset()

	def autonomousPeriodic(self):
		self.drive_for_time(0.4, 2.5)

	def teleopInit(self):
		pass
	
	def teleopPeriodic(self):
		pass

	def drive_for_time(self, speed, duration):
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
			return True

		return False

if __name__ == "__main__":
	wpilib.run(MyRobot)
