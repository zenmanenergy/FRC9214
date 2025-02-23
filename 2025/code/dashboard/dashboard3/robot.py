import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		""" Initialization code runs once when the robot powers on. """
		NetworkTables.initialize(server="roborio-9214-frc.local")
		self.table = NetworkTables.getTable("robot_data")

		# Robot state
		self.x_position = 0
		self.y_position = 0
		self.elevator_position = 0
		self.arm_angle = 0
		self.wrist_angle = 0
		self.grabber_angle = 0

	def autonomousPeriodic(self):
		""" Periodically update NetworkTables with real robot state. """
		# Simulate movement (Replace with real sensor values)
		self.x_position += 0.1
		self.y_position += 0.1
		self.elevator_position += 0.5 if self.elevator_position < 100 else 0

		# Send real-time data to NetworkTables
		self.table.putNumber("x_position", self.x_position)
		self.table.putNumber("y_position", self.y_position)
		self.table.putNumber("elevator", self.elevator_position)
		self.table.putNumber("arm_angle", self.arm_angle)
		self.table.putNumber("wrist_angle", self.wrist_angle)
		self.table.putNumber("grabber_angle", self.grabber_angle)

		# Read dashboard commands
		self.elevator_position = self.table.getNumber("elevator", self.elevator_position)
		self.arm_angle = self.table.getNumber("arm_angle", self.arm_angle)
		self.wrist_angle = self.table.getNumber("wrist_angle", self.wrist_angle)
		self.grabber_angle = self.table.getNumber("grabber_angle", self.grabber_angle)

		print(f"Position: ({self.x_position}, {self.y_position}), Elevator: {self.elevator_position}, Arm: {self.arm_angle}, Wrist: {self.wrist_angle}, Grabber: {self.grabber_angle}")

if __name__ == "__main__":
	wpilib.run(MyRobot)
