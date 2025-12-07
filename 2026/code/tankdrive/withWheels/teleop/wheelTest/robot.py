import wpilib
from drive import Drive

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.drive = Drive()
		self.joystick = wpilib.Joystick(0)  # Using a standard joystick

	def teleopPeriodic(self):
		# Fixed motor speed
		fixed_speed = .5

		# left control (Left stick up/down)
		left_speed = 0
		if self.joystick.getRawAxis(1) < -0.1:  # Up
			left_speed = fixed_speed
		elif self.joystick.getRawAxis(1) > 0.1:  # Down
			left_speed = -fixed_speed

		# right control (Right stick up/down)
		right_speed = 0
		if self.joystick.getRawAxis(5) < -0.1:  # Up
			right_speed = fixed_speed
		elif self.joystick.getRawAxis(5) > 0.1:  # Down
			right_speed = -fixed_speed


		# drive motors
		self.arm.drive_motors(left_speed, right_speed)

if __name__ == "__main__":
	wpilib.run(MyRobot)
