import wpilib
import ctre

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)
		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)
		
		# Initialize a timer and a step counter
		self.timer = wpilib.Timer()
		self.driveStep = 0  # Tracks which drive sequence we're on

	def autonomousInit(self):
		# Reset the step counter at the start of autonomous
		self.driveStep = 0

	def autonomousPeriodic(self):
		# Call the drive function based on the current drive step
		if self.driveStep == 0:
			# First drive forward for 0.5 seconds at 50% speed
			if self.drive_forward_for_time(0.5, 0.5):
				self.driveStep += 1  # Move to the next step

		elif self.driveStep == 1:
			# Second drive forward for 1 second at 75% speed
			if self.drive_forward_for_time(0.75, 1.0):
				self.driveStep += 1  # Move to the next step

	def teleopInit(self):
		pass
	
	def teleopPeriodic(self):
		pass

	def drive_forward_for_time(self, speed, duration):
		"""Drives forward at the given speed for the specified duration in seconds.
		Returns True once the duration has passed, otherwise False."""
		# Start driving if the timer hasn't started yet
		if self.timer.get() == 0:
			self.timer.start()
			self.LeftFrontMotor.set(-speed)
			self.LeftRearMotor.set(-speed)
			self.RightFrontMotor.set(speed)
			self.RightRearMotor.set(speed)

		# Check if the duration has passed
		if self.timer.hasPeriodPassed(duration):
			# Stop the motors
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			
			# Reset the timer for the next call
			self.timer.stop()
			self.timer.reset()
			return True  # Signal that the drive action is complete

		return False  # Drive action is still in progress

if __name__ == "__main__":
	wpilib.run(MyRobot)
