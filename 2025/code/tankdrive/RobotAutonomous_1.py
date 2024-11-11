import wpilib
import ctre

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)
		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)
		
		# Initialize a timer and a completion flag
		self.timer = wpilib.Timer()
		self.driveCompleted = False

	def autonomousInit(self):
		# Reset the drive completion flag at the start of autonomous
		self.driveCompleted = False

	def autonomousPeriodic(self):
		# Call the drive function only if drive is not completed
		if not self.driveCompleted:
			self.drive_forward_for_time(0.5, 0.5)

	def teleopInit(self):
		pass
	
	def teleopPeriodic(self):
		pass

	def drive_forward_for_time(self, speed, duration):
		"""Drives forward at the given speed for the specified duration in seconds."""
		# Start driving if not already driving
		if not self.timer.hasPeriodPassed(duration):
			# Start the timer and set motor speeds the first time
			if self.timer.get() == 0:
				self.timer.start()
				self.LeftFrontMotor.set(-speed)
				self.LeftRearMotor.set(-speed)
				self.RightFrontMotor.set(speed)
				self.RightRearMotor.set(speed)
		else:
			# Stop the motors and mark drive as completed
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			
			# Mark the drive as completed to prevent it from running again
			self.driveCompleted = True

if __name__ == "__main__":
	wpilib.run(MyRobot)
