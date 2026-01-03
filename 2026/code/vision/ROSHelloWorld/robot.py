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
		
		self.last_message = ""
	
	def teleopPeriodic(self):
		try:
			message = self.ros_table.getString("message", "")
			if message and message != self.last_message:
				print(f"[RoboRIO] Received: {message}")
				self.last_message = message
		except Exception as e:
			print(f"[RoboRIO] Error reading message: {e}")
		
if __name__ == "__main__":
	wpilib.run(MyRobot)

