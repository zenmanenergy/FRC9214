"""
Autonomous 2:
This program defines a more advanced FRC robot autonomous sequence using WPILib and CTRE libraries.
The robot executes a series of timed drive actions during the autonomous phase. The sequence includes
driving forward and backward at specified speeds and durations. A step counter tracks the progress
of the autonomous routine.
"""

from navx import AHRS
import wpilib
from wpimath.controller import PIDController
from phoenix5 import WPI_TalonSRX, NeutralMode

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.LeftFrontMotor = WPI_TalonSRX(1)
		self.LeftRearMotor = WPI_TalonSRX (4)

		self.RightFrontMotor = WPI_TalonSRX(3)
		self.RightRearMotor = WPI_TalonSRX(2)
		self.timer = wpilib.Timer()
		self.brakingTimer = wpilib.Timer()
		self.driveStep = 0
		self.RightFrontMotor.setNeutralMode(NeutralMode.Brake)
		self.LeftFrontMotor.setNeutralMode(NeutralMode.Brake)
		self.RightRearMotor.setNeutralMode(NeutralMode.Brake)
		self.LeftRearMotor.setNeutralMode(NeutralMode.Brake)
		self.LeftFrontMotor.setInverted(True)
		self.LeftRearMotor.setInverted(True)

		# NavX Sensor
		self.navx = AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k100Hz)  # Replace with your connection type if different


	def autonomousInit(self):
		self.startingCompass = self.navx.getCompassHeading()
		self.timer.stop()
		self.timer.reset()
		self.brakingTimer.stop()
		self.brakingTimer.reset()
		self.driveStep = 0
		# self.drive_forward_for_time(1, 1)

	def autonomousPeriodic(self):
		compass_heading = self.navx.getCompassHeading() - self.startingCompass
		print(f"Compass Heading: {compass_heading} degrees")


		# if self.driveStep == 0:
		# 	if self.drive_forward_for_time(0.4, 2.4):
		# 		self.driveStep += 1

		# elif self.driveStep == 1:
		# 	if self.drive_forward_for_time(-0.1, 0.25):
		# 		self.driveStep += 1

		# if self.driveStep == 0:
		# 	if self.turn(.2, 10):
		# 		self.driveStep += 1

		# elif self.driveStep == 3:
		# 	if self.turn(.05, 0.25):
		# 		self.driveStep += 1

		# elif self.driveStep == 4:
		# 	if self.drive_forward_for_time(0.4, 2.75):
		# 		self.driveStep += 1

		# elif self.driveStep == 5:
		# 	if self.drive_forward_for_time(-0.1, 0.25):
		# 		self.driveStep += 1

		# elif self.driveStep == 6:
		# 	if self.turn(-.4, 0.4):
		# 		self.driveStep += 1

	def teleopInit(self):
		pass
	
	def teleopPeriodic(self):
		pass

	def allMotorsStraight(self, speed):
		self.LeftFrontMotor.set(speed)
		self.LeftRearMotor.set(speed)
		self.RightFrontMotor.set(speed)
		self.RightRearMotor.set(speed)
		pass

	def drive_forward_for_time(self, speed, duration):
		if self.timer.get() == 0:
			self.timer.start()
			self.allMotorsStraight(speed)

		if self.timer.get() >= duration:
			# if braking:
			# 	if self.brakingTimer.get() == 0:
			# 		self.brakingTimer.start()
			# 		self.allMotorsTurn(-0.01)
			# 		return False
			# 	elif self.brakingTimer.get() >= .25:
			# 		self.allMotorsStraight(0)
			# 		self.brakingTimer.stop()
			# 		self.brakingTimer.reset()
			# 		self.timer.stop()
			# 		self.timer.reset()
			# 		return True

			# else:
			self.allMotorsStraight(0)
			self.timer.stop()
			self.timer.reset()
			return True
		
		return False

	def turn(self, speed, degree):

		current_heading = self.navx.getCompassHeading()

		error = degree - current_heading

		if error > 180:
			error -=360
		elif error < -180:
			error +=360

		turn_output = 0.02 * error * speed

		self.LeftFrontMotor.set(turn_output)
		self.RightFrontMotor.set(-turn_output)
		self.LeftRearMotor.set(turn_output)
		self.RightRearMotor.set(-turn_output)

		if abs(error) < 2:
			self.allMotorsStraight(0)
			return True

		
		# if self.timer.get() == 0:
		# 	self.timer.start()
		# 	self.LeftFrontMotor.set(speed)
		# 	self.LeftRearMotor.set(speed)
		# 	self.RightFrontMotor.set(speed)
		# 	self.RightRearMotor.set(speed)

		# if self.timer.get() >= duration:
		# 	# if braking:
		# 	# 	if self.brakingTimer.get() == 0:
		# 	# 		self.brakingTimer.start()
		# 	# 		self.allMotorsStraight(-0.1)
		# 	# 		return False
		# 	# 	elif self.brakingTimer.get() >= .25:
		# 	# 		self.allMotorsStraight(0)
		# 	# 		self.brakingTimer.stop()
		# 	# 		self.brakingTimer.reset()
		# 	# 		self.timer.stop()
		# 	# 		self.timer.reset()
		# 	# 		return True
		# 	# else:
		# 	self.allMotorsStraight(0)
		# 	self.timer.stop()
		# 	self.timer.reset()
		# 	return True
		
		return False



if __name__ == "__main__":
	wpilib.run(MyRobot)
