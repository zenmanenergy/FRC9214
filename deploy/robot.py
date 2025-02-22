import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		"""Initialization code runs once when the robot powers on."""
		NetworkTables.initialize(server="roborio-9214-frc.local")
		table = NetworkTables.getTable("arm")

		table.putNumber("elevator", 13)

		print(table.getNumber("elevator", 0))

	def teleopPeriodic(self):
		"""Periodic code that runs during teleoperated control."""
		pass
	 

if __name__ == "__main__":
	wpilib.run(MyRobot)
