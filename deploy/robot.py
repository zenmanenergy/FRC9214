import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		"""Initialization code runs once when the robot powers on."""
		NetworkTables.initialize(server="roborio-9214-frc.local")
		table = NetworkTables.getTable("vision")

		table.putNumber("target_x", 10)

		print(table.getNumber("target_x", 0))

	def teleopPeriodic(self):
		"""Periodic code that runs during teleoperated control."""
		pass
	 

if __name__ == "__main__":
	pass
