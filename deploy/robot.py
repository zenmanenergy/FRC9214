import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		"""Initialization code runs once when the robot powers on."""
		NetworkTables.initialize(server="roborio-9214-frc.local")
		table = NetworkTables.getTable("vision")

		print(table.getNumber("target_x", 0))

	def teleopPeriodic(self):
		"""Periodic code that runs during teleoperated control."""
		pass
	
	def periodic(self):
		print("hello")
		print("there")


if __name__ == "__main__":
	pass
