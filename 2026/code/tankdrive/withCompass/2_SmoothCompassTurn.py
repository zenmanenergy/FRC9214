from navx import AHRS
import wpilib
from wpimath.controller import PIDController
from phoenix5 import WPI_TalonSRX, NeutralMode

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Initialize motors
		self.leftFrontMotor = WPI_TalonSRX(1)
		self.leftRearMotor = WPI_TalonSRX(4)
		self.rightFrontMotor = WPI_TalonSRX(3)
		self.rightRearMotor = WPI_TalonSRX(2)
		
		# Invert left motors so that both sides move forward together
		self.leftFrontMotor.setInverted(True)
		self.leftRearMotor.setInverted(True)
		
		# Set all motors to brake mode
		self.leftFrontMotor.setNeutralMode(NeutralMode.Brake)
		self.leftRearMotor.setNeutralMode(NeutralMode.Brake)
		self.rightFrontMotor.setNeutralMode(NeutralMode.Brake)
		self.rightRearMotor.setNeutralMode(NeutralMode.Brake)
		
		# Timer for driving forward
		self.driveTimer = wpilib.Timer()
		
		# Initialize the NavX sensor
		self.navx = AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k200Hz)
		
		# Set up a PID controller for turning. Adjust kP, kI, and kD as needed.
		self.turnPID = PIDController(0.02, 0.0, 0.0)
		self.turnPID.enableContinuousInput(-180, 180)
		self.turnPID.setTolerance(1.0)
		
		self.driveStep = 0

	def autonomousInit(self):
		self.driveTimer.reset()
		self.driveStep = 0
		self.turnPID.reset()

	def autonomousPeriodic(self):
		# Step 0: Turn to 124 degrees
		if self.driveStep == 0:
			if self.turn_to_angle(124):
				self.driveStep += 1
				self.driveTimer.reset()
		# Step 1: Drive forward for 1 second at 0.1 speed
		elif self.driveStep == 1:
			if self.drive_forward_for_time(0.1, 1.0):
				self.driveStep += 1
		# Step 2: Turn to 10 degrees
		elif self.driveStep == 2:
			if self.turn_to_angle(10):
				self.driveStep += 1

	def teleopInit(self):
		pass
	
	def teleopPeriodic(self):
		pass

	def set_all_motors(self, speed):
		self.leftFrontMotor.set(speed)
		self.leftRearMotor.set(speed)
		self.rightFrontMotor.set(speed)
		self.rightRearMotor.set(speed)

	def drive_forward_for_time(self, speed, duration):
		# Start timer on first call
		if not self.driveTimer.hasStarted():
			self.driveTimer.start()
		self.set_all_motors(speed)
		if self.driveTimer.get() >= duration:
			self.set_all_motors(0)
			self.driveTimer.stop()
			self.driveTimer.reset()
			return True
		return False

	def turn_to_angle(self, target_angle):
		# Update the PID setpoint every cycle
		self.turnPID.setSetpoint(target_angle)
		output = self.turnPID.calculate(self.navx.getYaw())
		
		# For tank drive: set left side to negative output, right side to positive output
		self.leftFrontMotor.set(-output)
		self.leftRearMotor.set(-output)
		self.rightFrontMotor.set(output)
		self.rightRearMotor.set(output)
		
		# If the PID controller is within tolerance, stop the motors
		if self.turnPID.atSetpoint():
			self.set_all_motors(0)
			return True
		return False

if __name__ == "__main__":
	wpilib.run(MyRobot)
