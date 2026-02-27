"""Web Dashboard for Swerve Drive Calibration"""
from flask import Flask, render_template, jsonify
import ntcore
import threading
import time

app = Flask(__name__)

class DashboardServer:
	def __init__(self):
		self.wheel_angles = {
			"front_right": 0,
			"rear_right": 0,
			"rear_left": 0,
			"front_left": 0
		}
		self.counter = 0
		self.running = False
		self.table = None  # Initialize to None
	
	def start_nt_listener(self):
		"""Start listening to NetworkTables"""
		try:
			# Get default NetworkTables instance
			inst = ntcore.NetworkTableInstance.getDefault()
			inst.startClient4("WebDashboard")
			inst.setServerTeam(9214)
			self.table = inst.getTable("SmartDashboard")
			print("[NT] Connecting to NetworkTables team 9214...")
		except Exception as e:
			print(f"[NT] Error initializing: {e}")
			self.table = None
		
		self.running = True
		threading.Thread(target=self._update_values, daemon=True).start()
	
	def _update_values(self):
		"""Continuously read values from NetworkTables"""
		first_read = True
		
		while self.running:
			if self.table:
				try:
					self.wheel_angles["front_right"] = self.table.getNumber("FR Angle", 0)
					self.wheel_angles["rear_right"] = self.table.getNumber("RR Angle", 0)
					self.wheel_angles["rear_left"] = self.table.getNumber("RL Angle", 0)
					self.wheel_angles["front_left"] = self.table.getNumber("FL Angle", 0)
					self.counter = self.table.getNumber("Counter", 0)
					
					if first_read and self.counter > 0:
						print(f"[NT] Connected to NetworkTables - reading wheel angles")
						first_read = False
						
				except Exception as e:
					pass
			time.sleep(0.05)  # Update 20x per second

dashboard = DashboardServer()

# Start NetworkTables listener immediately when module loads
print("[WEB] Starting Web Dashboard...")
dashboard.start_nt_listener()

@app.route("/")
def index():
	return render_template("dashboard.html")

@app.route("/api/angles")
def get_angles():
	"""API endpoint for wheel angles"""
	return jsonify({
		"angles": dashboard.wheel_angles,
		"counter": dashboard.counter
	})

@app.route("/api/command", methods=["POST"])
def send_command():
	"""API endpoint to send commands to robot via NetworkTables"""
	from flask import request
	
	if not dashboard.table:
		return jsonify({"status": "error", "message": "Not connected to NetworkTables"}), 503
	
	cmd = request.json.get("cmd")
	value = request.json.get("value")
	
	try:
		if cmd == "focus":
			dashboard.table.putString("focused_wheel", value)
			print(f"[COMMAND] Focus: {value}")
		elif cmd == "power":
			dashboard.table.putNumber("wheel_power", float(value))
		elif cmd == "align":
			dashboard.table.putBoolean("align_command", True)
			print("[COMMAND] Align all wheels")
		elif cmd == "save_zero":
			dashboard.table.putBoolean("save_zero_command", True)
			print(f"[COMMAND] Save zero for {value}")
		
		return jsonify({"status": "ok", "command": cmd})
	except Exception as e:
		print(f"[COMMAND] Error: {e}")
		return jsonify({"status": "error", "message": str(e)}), 400

if __name__ == "__main__":
	print("[WEB] Starting NetworkTables listener...")
	dashboard.start_nt_listener()
	print("[WEB] Starting web server on http://localhost:5000")
	app.run(debug=False, host="0.0.0.0", port=5000)
