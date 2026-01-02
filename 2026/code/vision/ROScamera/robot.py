import wpilib
from networktables import NetworkTables

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Initialize NetworkTables client to connect to the Ubuntu computer (10.92.14.100)
		NetworkTables.initialize(server="10.92.14.200", port=5800)
		self.ros_table = NetworkTables.getTable("ros_data")
		
		print("[RoboRIO] Connecting to NetworkTables at 10.92.14.100:5800")
	
	def teleopPeriodic(self):
		# Retrieve and print the message from the Ubuntu NetworkTables server
		message = self.ros_table.getString("message", "")
		if message:
			print(f"[RoboRIO] Received: {message}")
		
if __name__ == "__main__":
	wpilib.run(MyRobot)

