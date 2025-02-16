import wpilib
from ntcore import NetworkTableInstance
import json

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# ✅ Initialize NetworkTables
		self.nt_inst = NetworkTableInstance.getDefault()
		self.nt_inst.startClientTeam(9214)  # Replace with your team number
		self.vision_table = self.nt_inst.getTable("Vision")

		# ✅ Variables to store received position data
		self.robot_x = 0.0
		self.robot_y = 0.0
		self.robot_heading = 0.0

	def teleopPeriodic(self):
		# ✅ Get JSON string from NetworkTables
		json_data = self.vision_table.getString("robot_position", "")

		if json_data:
			try:
				# ✅ Parse JSON data
				position_data = json.loads(json_data)

				# ✅ Extract x, y, heading values
				self.robot_x = position_data.get("x", 0.0)
				self.robot_y = position_data.get("y", 0.0)
				self.robot_heading = position_data.get("heading", 0.0)

				# ✅ Print debug info
				wpilib.SmartDashboard.putNumber("Robot X", self.robot_x)
				wpilib.SmartDashboard.putNumber("Robot Y", self.robot_y)
				wpilib.SmartDashboard.putNumber("Robot Heading", self.robot_heading)
				print(f"Received: X={self.robot_x:.2f}, Y={self.robot_y:.2f}, Heading={self.robot_heading:.1f}°")

			except json.JSONDecodeError:
				print("⚠️ Error decoding JSON from NetworkTables!")

if __name__ == "__main__":
	wpilib.run(MyRobot)
