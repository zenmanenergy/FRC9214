import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		"""Initialization code runs once when the robot powers on."""
		NetworkTables.initialize(server="roborio-9214-frc.local")
		self.table = NetworkTables.getTable("arm")
		self.direction='up'
		self.elevator_position=0
	def autonomousPeriodic(self):
		if self.direction is "up":
			self.elevator_position+=0.5
		else:
			self.elevator_position-=0.5
		self.table.putNumber("elevator", self.elevator_position)
		if self.elevator_position>=100:
			self.direction='down'
		elif self.elevator_position<=0:
			self.direction='up'

		print(self.direction,self.elevator_position)

		
	 

if __name__ == "__main__":
	wpilib.run(MyRobot)
