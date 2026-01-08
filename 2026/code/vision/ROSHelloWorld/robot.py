import wpilib
from networktables import NetworkTables
import time

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		print("[RoboRIO] Starting initialization...")
		try:
			NetworkTables.initialize(server="10.92.14.200")
			self.ros_table = NetworkTables.getTable("ros_data")
			print("[RoboRIO] NetworkTables connected!")
		except Exception as e:
			print(f"[RoboRIO] Connection failed: {e}")
		print("Robot Initialization")
		# Initialize NetworkTables client to connect to the Ubuntu computer (10.92.14.200)
		NetworkTables.initialize(server="10.92.14.200")
		self.ros_table = NetworkTables.getTable("ros_data")
		
		print("[RoboRIO] Connecting to NetworkTables at 10.92.14.200")
	
	def teleopPeriodic(self):
		
		# Retrieve and print the message from the Ubuntu NetworkTables server
		message = self.ros_table.getString("message", "")
		
		if message:
			print(f"[RoboRIO] Received: {message}")
		
if __name__ == "__main__":
	wpilib.run(MyRobot)

