import wpilib # Imports WPILIB library (FIRST Robotics robot base classes and hardware APIs)
from phoenix5 import WPI_TalonSRX, NeutralMode # Imports WPI_TalonSRX and NeutralMode from Phoenix5 library (CTRE Talon SRX motor controller APIs)
import rev # Imports REV library (REV Robotics SPARK MAX APIs)
from arm import Arm # Imports class "Arm" from arm.py
from drive import Drive # Imports class "Drive" from drive.py
from auto import Auto # Imports class "Auto" from auto.py

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		
		# Creates motor controller objects for each of the four drive motors. 
		left_front = WPI_TalonSRX(4)
		left_rear = WPI_TalonSRX(2)
		right_front = WPI_TalonSRX(3)
		right_rear = WPI_TalonSRX(1)

		# Inverts right-side motors so a positive speed makes both sides drive forward
		right_front.setInverted(True)
		right_rear.setInverted(True)

		# Sets all drive motors to brake when no power is applied (robot resists coasting)
		left_front.setNeutralMode(NeutralMode.Brake)
		left_rear.setNeutralMode(NeutralMode.Brake)
		right_front.setNeutralMode(NeutralMode.Brake)
		right_rear.setNeutralMode(NeutralMode.Brake)

		# Creates motor controllers for elevator, shoulder, wrist, and grabber joints	
		elevator_motor = rev.SparkMax(10, rev.SparkMax.MotorType.kBrushless)
		shoulder_motor = rev.SparkMax(12, rev.SparkMax.MotorType.kBrushless)
		wrist_motor = rev.SparkMax(11, rev.SparkMax.MotorType.kBrushless)
		grabber_motor = rev.SparkMax(13, rev.SparkMax.MotorType.kBrushless)

		# Sets motor inversion so each joint moves in the correct direction for joystick input
		elevator_motor.setInverted(True)
		shoulder_motor.setInverted(True)
		wrist_motor.setInverted(False)
		grabber_motor.setInverted(True)
		
		# Assigns arm control to controller 0 and drive controls to controller 1
		ArmJoystick = wpilib.Joystick(0)
		DriveJoystick = wpilib.Joystick(1)
		
		# Creates Arm, Drive, and Auto subsystem objects using the motors and joysticks
		self.arm = Arm(ArmJoystick, elevator_motor, shoulder_motor, wrist_motor, grabber_motor)
		self.drive = Drive(DriveJoystick,left_front,left_rear,right_front,right_rear)
		self.auto= Auto(wpilib, left_front,left_rear,right_front,right_rear)

		# Tracks whether subsystems have been reset/zeroed since last enable
		self.Zeroed=False

	# On disable, reset all subsystems and mark them as not yet zeroed for next mode
	def disabledInit(self):
		self.Zeroed=False
		self.arm.reset()
		self.drive.reset()
		self.auto.reset()
	
	# At the start of autonomous, reset subsystems once if they haven't been zeroed yet, then start auto routine
	def autonomousInit(self):
		if not self.Zeroed:
			self.arm.reset()
			self.drive.reset()
			self.auto.reset()
			self.Zeroed = True

		self.auto.start(speed=0.5, duration=2.0)

	# Runs autonomous actions every robot loop during autonomous mode
	def autonomousPeriodic(self):
		self.auto.periodic()

	# At start of teleop, reset arm and drive once if they haven't been zeroed yet
	def teleopInit(self):
		if not self.Zeroed:
			self.arm.reset()
			self.drive.reset()
			self.Zeroed = True
	
	# Runs drive and arm control code every robot loop during teleop
	def teleopPeriodic(self):
		self.drive.periodic()
		self.arm.periodic(False)
		
# Entry point: starts the robot using MyRobot as the main robot class
if __name__ == "__main__":
	wpilib.run(MyRobot)