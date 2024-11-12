import wpilib

class Robot(wpilib.TimedRobot):
	def robotInit(self):
		# Initialize the limit switch on a DIO port (e.g., DIO0)
		self.limit_switch = wpilib.DigitalInput(0)

	def teleopPeriodic(self):
		# Check if the limit switch is pressed
		if self.limit_switch.get() == False:  # Assuming False means pressed
			print("Limit switch is pressed.")
		else:
			print("Limit switch is not pressed.")

if __name__ == "__main__":
	wpilib.run(Robot)
