import wpilib
from Drive import Drive

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.joystick = wpilib.Joystick(0)
		self.drive = Drive()
		# For example, assume the robot starts at (0, 0) in mm.
		self.current_x = 0
		self.current_y = 0

	def disabledInit(self):
		print("bot is disabled")
		self.drive.reset()
	def teleopInit(self):
		self.joystick = wpilib.Joystick(0)
		self.drive.reset_encoders()
		self.drive.traveling = False
		self.drive.turning = False
		self.drive.turning = False
		self.current_x = 0
		self.current_y = 0
		self.drive.reset()
	def teleopPeriodic(self):
		self.drive.traveling = False
		self.drive.turning = False
		self.drive.turning = False
		self.current_x = 0
		self.current_y = 0

	def teleopPeriodic(self):
		# If not already executing a travel or turn command...
		if not self.drive.traveling and not self.drive.turning and not self.drive.navigating:
			if self.joystick.getRawButtonReleased(4):  # Y button to move forward 1000 mm
				print("Y button")
				self.drive.setCoordinates(0,0)

				self.drive.start_travel(4000)
			elif self.joystick.getRawButtonReleased(1):  # A button to move backward 1000 mm
				print("A button")
				self.drive.start_travel(-4000)
			elif self.joystick.getRawButtonReleased(3):  # X button to turn left 90 degrees
				print("X button")
				self.drive.start_turn(-90, 0.3)
			elif self.joystick.getRawButtonReleased(2):  # B button to turn right 90 degrees
				print("B button")
				self.drive.start_turn(90, 0.3)
			elif self.joystick.getRawButtonReleased(5):  # (LB button) (button 5) for navigation
				print("LB button")
				# For example, navigate from current position to (3000, 2000) mm.
				self.drive.start_navigation(0, 0, 0, 4000)
				
				# Optionally update your current coordinate once navigation is complete.
			elif self.joystick.getRawButtonReleased(6):
				print("RB button")
				print("Current Coordinates:", self.drive.getCoordinates())
				self.drive.printHeading()
			elif self.joystick.getRawButtonReleased(8):
				print("Start button")
				self.drive.reset()
		# Update travel, turn, or navigation if active.
		if self.drive.traveling:
			self.drive.traveling = self.drive.update_travel()
			print("Current Coordinates:", self.drive.getCoordinates())
			print("Heading:", self.drive.getHeading())
		if self.drive.turning:
			self.drive.turning = self.drive.update_turn()
		if self.drive.navigating:
			# update_navigation() returns True if still navigating.
			navigating = self.drive.update_navigation()
			# Optionally, once navigation is complete, update current_x and current_y.
			# if not navigating:
			# 	# For example, set current position to the target (hard-coded here for demo).
			# 	self.current_x = 3000
			# 	self.current_y = 2000

if __name__ == "__main__":
	wpilib.run(MyRobot)
