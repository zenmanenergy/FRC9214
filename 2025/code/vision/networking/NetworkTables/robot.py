import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		"""Initialization code runs once when the robot powers on."""
		NetworkTables.initialize()

		table = NetworkTables.getTable("vision")

		table.putNumber("x", 10)

		print(table.getNumber("x"))

	def teleopPeriodic(self):
		"""Periodic code that runs during teleoperated control."""
		

if __name__ == "__main__":
	pass
