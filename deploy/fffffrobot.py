import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		"""Initialization code runs once when the robot powers on."""
		# NetworkTables.initialize(server="roborio-9214-frc.local")
		NetworkTables.initialize(server="10.92.14.2")
		self.table = NetworkTables.getTable("vision")

		print(self.table.getNumber("target_x", 0))

	def teleopPeriodic(self):
		"""Periodic code that runs during teleoperated control."""
		print("x: " + str(self.table.getNumber("x",0)))
		print("y: " + str(self.table.getNumber("y",0)))
		print("heading: " + str(self.table.getNumber("heading",0)))
		# pass

if __name__ == "__main__":
	wpilib.run(MyRobot)



