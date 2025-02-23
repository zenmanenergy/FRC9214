import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Initialize NetworkTables
		NetworkTables.initialize(server="roborio-9214-frc.local")
		NetworkTables.initialize(server=192.168.1.125")  # Replace with your RoboRIO IP if needed
		self.table = NetworkTables.getTable("robot_data")

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

		# Print connection status every few cycles (avoid spamming)
		if self.getTime() % 2 < 0.02:  # Print every ~2 seconds
			if NetworkTables.isConnected():
				print("NetworkTables Connected!")
			else:
				print("NetworkTables Disconnected!")

		# Read dashboard commands
		self.cmd_elevator_position = self.table.getNumber("cmd_elevator", self.cmd_elevator_position)
		self.cmd_arm_angle = self.table.getNumber("cmd_arm_angle", self.cmd_arm_angle)
		self.cmd_wrist_angle = self.table.getNumber("cmd_wrist_angle", self.cmd_wrist_angle)
		self.cmd_grabber_angle = self.table.getNumber("cmd_grabber_angle", self.cmd_grabber_angle)

		# Print received commands
		print(f"CMD -> Elevator: {self.cmd_elevator_position}, Arm: {self.cmd_arm_angle}, Wrist: {self.cmd_wrist_angle}, Grabber: {self.cmd_grabber_angle}")

		# Apply commanded values to real robot movement
		if abs(self.real_elevator_position - self.cmd_elevator_position) > 0.1:
			self.real_elevator_position += (self.cmd_elevator_position - self.real_elevator_position) * 0.2
		if abs(self.real_arm_angle - self.cmd_arm_angle) > 0.1:
			self.real_arm_angle += (self.cmd_arm_angle - self.real_arm_angle) * 0.2
		if abs(self.real_wrist_angle - self.cmd_wrist_angle) > 0.1:
			self.real_wrist_angle += (self.cmd_wrist_angle - self.real_wrist_angle) * 0.2
		if abs(self.real_grabber_angle - self.cmd_grabber_angle) > 0.1:
			self.real_grabber_angle += (self.cmd_grabber_angle - self.real_grabber_angle) * 0.2


		# Print real positions to confirm updates
		print(f"REAL -> X: {self.real_x_position}, Y: {self.real_y_position}, Elevator: {self.real_elevator_position}, Arm: {self.real_arm_angle}, Wrist: {self.real_wrist_angle}, Grabber: {self.real_grabber_angle}")

		# Send real-time data (real state)
		self.table.putNumber("real_x_position", self.real_x_position)
		self.table.putNumber("real_y_position", self.real_y_position)
		self.table.putNumber("real_elevator", self.real_elevator_position)
		self.table.putNumber("real_arm_angle", self.real_arm_angle)
		self.table.putNumber("real_wrist_angle", self.real_wrist_angle)
		self.table.putNumber("real_grabber_angle", self.real_grabber_angle)

if __name__ == "__main__":
	wpilib.run(MyRobot)
