import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Initialize NetworkTables with correct IP
		NetworkTables.initialize(server="192.168.1.125")  # Use actual RoboRIO IP
		self.table = NetworkTables.getTable("robot_data")

		# Real robot state (from sensors/motors)
		self.real_x_position = 0
		self.real_y_position = 0
		self.real_elevator_position = 0
		self.real_arm_angle = 0
		self.real_wrist_angle = 0
		self.real_grabber_angle = 0

		# Commanded state (received from dashboard)
		self.cmd_elevator_position = None  # None means "no new command"
		self.cmd_arm_angle = None
		self.cmd_wrist_angle = None
		self.cmd_grabber_angle = None

	def teleopPeriodic(self):
		""" Read commands from NetworkTables but only apply them if real sensors report changes. """

		# Read dashboard commands
		new_cmd_elevator_position = self.table.getNumber("cmd_elevator", self.cmd_elevator_position)
		new_cmd_arm_angle = self.table.getNumber("cmd_arm_angle", self.cmd_arm_angle)
		new_cmd_wrist_angle = self.table.getNumber("cmd_wrist_angle", self.cmd_wrist_angle)
		new_cmd_grabber_angle = self.table.getNumber("cmd_grabber_angle", self.cmd_grabber_angle)

		# Only print if commands have changed
		if (new_cmd_elevator_position != self.cmd_elevator_position or
			new_cmd_arm_angle != self.cmd_arm_angle or
			new_cmd_wrist_angle != self.cmd_wrist_angle or
			new_cmd_grabber_angle != self.cmd_grabber_angle):

			print(f"CMD -> Elevator: {new_cmd_elevator_position}, Arm: {new_cmd_arm_angle}, Wrist: {new_cmd_wrist_angle}, Grabber: {new_cmd_grabber_angle}")

		# Update command values
		self.cmd_elevator_position = new_cmd_elevator_position
		self.cmd_arm_angle = new_cmd_arm_angle
		self.cmd_wrist_angle = new_cmd_wrist_angle
		self.cmd_grabber_angle = new_cmd_grabber_angle

		# ---- DO NOT CHANGE REAL VALUES UNLESS THE ROBOT REPORTS THEM ----
		updated = False

		# (These should come from real sensors in the future)
		new_real_elevator_position = self.real_elevator_position  # Replace with actual sensor reading
		new_real_arm_angle = self.real_arm_angle  # Replace with actual motor feedback
		new_real_wrist_angle = self.real_wrist_angle  # Replace with actual feedback
		new_real_grabber_angle = self.real_grabber_angle  # Replace with actual feedback

		# Only print when real values change
		if new_real_elevator_position != self.real_elevator_position:
			self.real_elevator_position = new_real_elevator_position
			updated = True
		if new_real_arm_angle != self.real_arm_angle:
			self.real_arm_angle = new_real_arm_angle
			updated = True
		if new_real_wrist_angle != self.real_wrist_angle:
			self.real_wrist_angle = new_real_wrist_angle
			updated = True
		if new_real_grabber_angle != self.real_grabber_angle:
			self.real_grabber_angle = new_real_grabber_angle
			updated = True

		# Print real updates only when something actually changed
		if updated:
			print(f"REAL -> Elevator: {self.real_elevator_position}, Arm: {self.real_arm_angle}, Wrist: {self.real_wrist_angle}, Grabber: {self.real_grabber_angle}")

		# Send real-time data to NetworkTables
		self.table.putNumber("real_x_position", self.real_x_position)
		self.table.putNumber("real_y_position", self.real_y_position)
		self.table.putNumber("real_elevator", self.real_elevator_position)
		self.table.putNumber("real_arm_angle", self.real_arm_angle)
		self.table.putNumber("real_wrist_angle", self.real_wrist_angle)
		self.table.putNumber("real_grabber_angle", self.real_grabber_angle)

if __name__ == "__main__":
	wpilib.run(MyRobot)
