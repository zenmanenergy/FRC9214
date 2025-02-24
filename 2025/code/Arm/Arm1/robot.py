import wpilib
from arm import Arm

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.arm = Arm()
		self.joystick = wpilib.Joystick(0)  # Using a standard joystick

	def teleopPeriodic(self):
		# Fixed motor speed
		fixed_speed = .5

		# Elevator control (Left stick up/down)
		elevator_speed = 0
		if self.joystick.getRawAxis(1) < -0.1:  # Up
			elevator_speed = fixed_speed
		elif self.joystick.getRawAxis(1) > 0.1:  # Down
			elevator_speed = -fixed_speed

		# Shoulder control (Right stick up/down)
		shoulder_speed = 0
		if self.joystick.getRawAxis(5) < -0.1:  # Up
			shoulder_speed = fixed_speed
		elif self.joystick.getRawAxis(5) > 0.1:  # Down
			shoulder_speed = -fixed_speed

		# Wrist control (Y and A buttons)
		wrist_speed = 0
		if self.joystick.getRawButton(4):  # Y button
			wrist_speed = fixed_speed  # Wrist Up
		elif self.joystick.getRawButton(1):  # A button
			wrist_speed = -fixed_speed  # Wrist Down

		# Grabber control (X and B buttons)
		grabber_speed = 0
		if self.joystick.getRawButton(3):  # X button
			grabber_speed = fixed_speed  # Intake
		elif self.joystick.getRawButton(2):  # B button
			grabber_speed = -fixed_speed  # Outtake

		# Control motors
		self.arm.control_motors(elevator_speed, shoulder_speed, wrist_speed, grabber_speed)

if __name__ == "__main__":
	wpilib.run(MyRobot)
