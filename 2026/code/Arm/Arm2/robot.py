import wpilib
from networktables import NetworkTables
from arm import Arm
from drive import Drive

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Initialize NetworkTables
		# NetworkTables.initialize(server="roborio-9214-frc.local")
		NetworkTables.initialize(server="10.92.14.2")
		self.table = NetworkTables.getTable("robot_data")

		self.arm = Arm(self.table)
		self.drive = Drive(self.table)


		# Check if connected to NetworkTables
		if NetworkTables.isConnected():
			print("Connected to NetworkTables!")
		else:
			print("NetworkTables Connection Failed!")

		# Robot state (real values from sensors/motors)
		self.real_x_position = 0
		self.real_y_position = 0
		self.real_elevator_position = 0
		self.real_arm_angle = 0
		self.real_wrist_angle = 0
		self.real_grabber_angle = 0

		# Commanded state (received from dashboard)
		self.cmd_elevator_position = 0
		self.cmd_arm_angle = 0
		self.cmd_wrist_angle = 0
		self.cmd_grabber_angle = 0

	def teleopPeriodic(self):
		""" Periodically update NetworkTables with real & commanded states. """

		

		# Read dashboard commands
		self.cmd_elevator_position = self.table.getNumber("cmd_elevator", self.cmd_elevator_position)
		self.cmd_arm_angle = self.table.getNumber("cmd_arm_angle", self.cmd_arm_angle)
		self.cmd_wrist_angle = self.table.getNumber("cmd_wrist_angle", self.cmd_wrist_angle)
		self.cmd_grabber_angle = self.table.getNumber("cmd_grabber_angle", self.cmd_grabber_angle)

		# Print received commands
		print(f"CMD -> Elevator: {self.cmd_elevator_position}, Arm: {self.cmd_arm_angle}, Wrist: {self.cmd_wrist_angle}, Grabber: {self.cmd_grabber_angle}")

		# Apply commanded values to real robot movement
		# if abs(self.real_elevator_position - self.cmd_elevator_position) > 0.1:
		# 	self.real_elevator_position += (self.cmd_elevator_position - self.real_elevator_position) * 0.2
		# if abs(self.real_arm_angle - self.cmd_arm_angle) > 0.1:
		# 	self.real_arm_angle += (self.cmd_arm_angle - self.real_arm_angle) * 0.2
		# if abs(self.real_wrist_angle - self.cmd_wrist_angle) > 0.1:
		# 	self.real_wrist_angle += (self.cmd_wrist_angle - self.real_wrist_angle) * 0.2
		# if abs(self.real_grabber_angle - self.cmd_grabber_angle) > 0.1:
		# 	self.real_grabber_angle += (self.cmd_grabber_angle - self.real_grabber_angle) * 0.2

		# Get real-time data (real state)
		self.arm.periodic(True)
		self.drive.periodic()



		# Print real positions to confirm updates
		

if __name__ == "__main__":
	wpilib.run(MyRobot)
