import wpilib
from Drive2 import Drive

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.joystick = wpilib.Joystick(0)
		self.drive = Drive()
		# For example, assume the robot starts at (0, 0) in mm.
		self.current_x = 0
		self.current_y = 0

	def teleopInit(self):
		self.drive.reset_encoders()
		self.drive.traveling = False
		self.drive.turning = False

	def teleopPeriodic(self):
		# If not already executing a travel or turn command...
		if not self.drive.traveling and not self.drive.turning and not self.drive.navigating:
			if self.joystick.getRawButton(4):  # Y button to move forward 4000 mm
				self.drive.setCoordinates(0,0)
				self.drive.start_travel(4000)
			elif self.joystick.getRawButton(1):  # A button to move backward 4000 mm
				self.drive.start_travel(-4000)
			elif self.joystick.getRawButton(3):  # X button to turn left 90 degrees
				self.drive.start_turn(-90, 0.4)
			elif self.joystick.getRawButton(2):  # B button to turn right 90 degrees
				self.drive.start_turn(90, 0.4)
			elif self.joystick.getRawButton(5):  # New button (button 5) for navigation
				# For example, navigate from current position to (3000, 2000) mm.
				self.drive.navigate_to(self.current_x, self.current_y, 3000, 2000)
				# Optionally update your current coordinate once navigation is complete.

		# Update travel, turn, or navigation if active.
		if self.drive.traveling:
			self.drive.traveling = self.drive.travel_distance()
			print("Current Coordinates:", self.drive.getCoordinates())
		if self.drive.turning:
			self.drive.turning = self.drive.update_turn()
		if self.drive.navigating:
			# update_navigation() returns True if still navigating.
			navigating = self.drive.update_navigation()
			# Optionally, once navigation is complete, update current_x and current_y.
			if not navigating:
				# For example, set current position to the target (hard-coded here for demo).
				self.current_x = 3000
				self.current_y = 2000

if __name__ == "__main__":
	wpilib.run(MyRobot)
