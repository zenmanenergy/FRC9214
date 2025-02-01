import wpilib
from Drive import Drive

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.joystick = wpilib.Joystick(0)
		self.drive = Drive()

	def teleopInit(self):
		self.drive.reset_encoders()
		self.drive.traveling = False
		self.drive.turning = False

	def teleopPeriodic(self):
		if not self.drive.traveling and not self.drive.turning:
			if self.joystick.getRawButton(4):  # Y button to move forward
				self.drive.start_travel(4000)
			elif self.joystick.getRawButton(1):  # A button to move backward
				self.drive.start_travel(-4000)
			elif self.joystick.getRawButton(3):  # X button to turn left 90 degrees
				self.drive.start_turn(-90,0.4)
			elif self.joystick.getRawButton(2):  # B button to turn right 90 degrees
				self.drive.start_turn(90,0.4)

		if self.drive.traveling:
			self.drive.traveling = self.drive.travel_distance()
		
		if self.drive.turning:
			self.drive.turning = self.drive.update_turn()

if __name__ == "__main__":
	wpilib.run(MyRobot)
