import wpilib
from talon import WPI_TalonSRX, NeutralMode, init_webots_socket

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Left motors
		self.left_front = WPI_TalonSRX(1)
		self.left_rear = WPI_TalonSRX(2)

		# Right motors
		self.right_front = WPI_TalonSRX(3)
		self.right_rear = WPI_TalonSRX(4)

		# Invert right motors
		self.right_front.setInverted(True)
		self.right_rear.setInverted(True)

		# Set all motors to brake mode
		self.left_front.setNeutralMode(NeutralMode.Brake)
		self.left_rear.setNeutralMode(NeutralMode.Brake)
		self.right_front.setNeutralMode(NeutralMode.Brake)
		self.right_rear.setNeutralMode(NeutralMode.Brake)

		 # Pre-initialize the Webots socket connection
		init_webots_socket()

		# Timer for autonomous
		self.auto_timer = wpilib.Timer()

	def robotPeriodic(self):
		# Override this method to quickly do periodic tasks to feed the watchdog.
		pass

	def autonomousInit(self):
		self.auto_timer.reset()
		self.auto_timer.start()

	def autonomousPeriodic(self):
		time = self.auto_timer.get()

		if time < 1.0:
			# Drive forward
			self.set_drive(0.5)
		elif time < 2.0:
			# Drive backward
			self.set_drive(-0.5)
		else:
			# Stop
			self.set_drive(0.0)

	def set_drive(self, speed):
		self.left_front.set(speed)
		self.left_rear.set(speed)
		self.right_front.set(speed)
		self.right_rear.set(speed)


if __name__ == "__main__":
	wpilib.run(MyRobot)
