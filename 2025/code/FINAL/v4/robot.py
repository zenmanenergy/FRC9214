import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode
import rev
from arm import Arm
from drive import Drive
from auto import Auto

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		
		left_front = WPI_TalonSRX(4)
		left_rear = WPI_TalonSRX(2)
		right_front = WPI_TalonSRX(3)
		right_rear = WPI_TalonSRX(1)

		right_front.setInverted(True)
		right_rear.setInverted(True)
		left_front.setNeutralMode(NeutralMode.Brake)
		left_rear.setNeutralMode(NeutralMode.Brake)
		right_front.setNeutralMode(NeutralMode.Brake)
		right_rear.setNeutralMode(NeutralMode.Brake)


		elevator_motor = rev.SparkMax(10, rev.SparkMax.MotorType.kBrushless)
		shoulder_motor = rev.SparkMax(12, rev.SparkMax.MotorType.kBrushless)
		wrist_motor = rev.SparkMax(11, rev.SparkMax.MotorType.kBrushless)
		grabber_motor = rev.SparkMax(13, rev.SparkMax.MotorType.kBrushless)

		elevator_motor.setInverted(True)
		shoulder_motor.setInverted(True)
		wrist_motor.setInverted(False)
		grabber_motor.setInverted(True)
		

		ArmJoystick = wpilib.Joystick(0)
		DriveJoystick = wpilib.Joystick(1)
		
		left_encoder = wpilib.Encoder(0, 1)
		right_encoder = wpilib.Encoder(8, 9)

		self.arm = Arm(wpilib,ArmJoystick, elevator_motor, shoulder_motor, wrist_motor, grabber_motor)
		self.drive = Drive(DriveJoystick, left_front, left_rear, right_front, right_rear, left_encoder, right_encoder)  # Fixed missing encoders
		self.auto= Auto(wpilib, left_front,left_rear,right_front,right_rear, self.arm, self.drive)

		self.Zeroed=False

	def disabledInit(self):
		self.arm.autoPreset = None
		self.arm.isAuto = False
		self.Zeroed=False
		# self.arm.reset()
		self.drive.reset()
		self.auto.reset()
		
	def autonomousInit(self):
		self.arm.autoPreset = None
		self.arm.isAuto = True
		if not self.Zeroed:
			self.arm.reset()
			self.drive.reset()
			self.auto.reset()
			self.Zeroed = True

		self.auto.start(speed=0.4, duration=1.5)
	def autonomousPeriodic(self):
		self.auto.periodic()
		self.arm.periodic()

	def teleopInit(self):
		self.arm.autoPreset = None
		self.arm.isAuto = False
		# print(self.Zeroed)
		if not self.Zeroed:
			# print("what")
			# self.arm.reset()
			# self.drive.reset()
			self.Zeroed = True
			
	def teleopPeriodic(self):
		self.drive.periodic()
		self.arm.periodic()
		

if __name__ == "__main__":
	wpilib.run(MyRobot)
