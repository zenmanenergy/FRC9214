import wpilib
from phoenix5 import WPI_TalonSRX, NeutralMode
from Drive5 import Drive

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):

		left_front = WPI_TalonSRX(4)
		left_rear = WPI_TalonSRX(2)
		right_front = WPI_TalonSRX(3)
		right_rear = WPI_TalonSRX(1)
		
		left_front.configVoltageCompSaturation(12)
		left_rear.configVoltageCompSaturation(12)
		right_front.configVoltageCompSaturation(12)
		right_rear.configVoltageCompSaturation(12)

		right_front.setInverted(True)
		right_rear.setInverted(True)
		left_front.setNeutralMode(NeutralMode.Brake)
		left_rear.setNeutralMode(NeutralMode.Brake)
		right_front.setNeutralMode(NeutralMode.Brake)
		right_rear.setNeutralMode(NeutralMode.Brake)
		left_encoder = wpilib.Encoder(0, 1)
		right_encoder = wpilib.Encoder(8, 9)

		DriveJoystick = wpilib.Joystick(1)
		self.drive = Drive(DriveJoystick, left_front, left_rear, right_front, right_rear, left_encoder, right_encoder)  # Fixed missing encoders
		
		

	def disabledInit(self):
		print("bot is disabled")
		self.drive.reset()

	def autonomousInit(self):
		self.drive.reset()
		self.drive.start_travel(1430)

	def autonomousPeriodic(self):
		self.drive.update_travel(base_speed=0.4)

		if not self.drive.traveling:
			print("RUN L4 UNLOAD!")


if __name__ == "__main__":
	wpilib.run(MyRobot)
