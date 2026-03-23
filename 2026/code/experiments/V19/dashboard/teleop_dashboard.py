"""Swerve Drive Teleop Dashboard - WebSocket Server (Match Mode Only)"""
from flask import Flask, render_template, request
import ntcore
import threading
import time
import json
import os

try:
	from flask_sock import Sock
	HAS_FLASK_SOCK = True
except ImportError:
	HAS_FLASK_SOCK = False
	print("[ERROR] flask-sock not installed. Run: pip install flask-sock")

template_dir = os.path.join(os.path.dirname(__file__), 'templates')
app = Flask(__name__, template_folder=template_dir)

if HAS_FLASK_SOCK:
	sock = Sock(app)

class DashboardServer:
	def __init__(self):
		self.wheel_angles = {
			"front_right": 0,
			"rear_right": 0,
			"rear_left": 0,
			"front_left": 0
		}
		self.wheel_power = {
			"front_right": 0,
			"rear_right": 0,
			"rear_left": 0,
			"front_left": 0
		}
		self.counter = 0
		self.robot_enabled = False
		self.robot_mode = "Unknown"
		self.running = False
		self.table = None
		self.ws_clients = set()
	
	def start_nt_listener(self):
		"""Start listening to NetworkTables"""
		try:
			inst = ntcore.NetworkTableInstance.getDefault()
			inst.startClient4("TeleopDashboard")
			inst.setServerTeam(9214)
			self.table = inst.getTable("SmartDashboard")
			print("[NT] TELEOP: Connecting to NetworkTables team 9214...")
		except Exception as e:
			print(f"[NT] Error initializing: {e}")
			self.table = None
		
		self.running = True
		threading.Thread(target=self._update_values, daemon=True).start()
	
	def register_ws_client(self, ws):
		"""Register a WebSocket client"""
		self.ws_clients.add(ws)
		print(f"[WS] Client connected. Total clients: {len(self.ws_clients)}")
		self._broadcast_state_to_client(ws)
	
	def unregister_ws_client(self, ws):
		"""Unregister a WebSocket client"""
		self.ws_clients.discard(ws)
		print(f"[WS] Client disconnected. Total clients: {len(self.ws_clients)}")
	
	def _broadcast_state_to_client(self, ws):
		"""Send current state to a specific client"""
		try:
			state = {
				"type": "state",
				"angles": self.wheel_angles,
				"power": self.wheel_power,
				"counter": self.counter,
				"robot_enabled": self.robot_enabled,
				"robot_mode": self.robot_mode
			}
			ws.send(json.dumps(state))
		except Exception as e:
			print(f"[WS] Error sending to client: {e}")
			self.ws_clients.discard(ws)
	
	def _broadcast_to_all(self, message_dict):
		"""Broadcast a message to all connected WebSocket clients"""
		msg_json = json.dumps(message_dict)
		disconnected = set()
		
		for ws in self.ws_clients:
			try:
				ws.send(msg_json)
			except Exception as e:
				print(f"[WS] Error broadcasting to client: {e}")
				disconnected.add(ws)
		
		for ws in disconnected:
			self.ws_clients.discard(ws)
	
	def _update_values(self):
		"""Continuously read values from NetworkTables and broadcast changes"""
		first_read = True
		
		while self.running:
			if self.table:
				try:
					new_angles = {
						"front_right": self.table.getNumber("FR Angle", 0),
						"rear_right": self.table.getNumber("RR Angle", 0),
						"rear_left": self.table.getNumber("RL Angle", 0),
						"front_left": self.table.getNumber("FL Angle", 0)
					}
					new_power = {
						"front_right": self.table.getNumber("FR Power", 0),
						"rear_right": self.table.getNumber("RR Power", 0),
						"rear_left": self.table.getNumber("RL Power", 0),
						"front_left": self.table.getNumber("FL Power", 0)
					}
					new_counter = self.table.getNumber("Counter", 0)
					new_robot_enabled = self.table.getBoolean("Robot Enabled", False)
					new_robot_mode = self.table.getString("robot_mode", "Unknown")
					
					if (new_angles != self.wheel_angles or new_power != self.wheel_power or new_counter != self.counter or 
						new_robot_enabled != self.robot_enabled or new_robot_mode != self.robot_mode):
						self.wheel_angles = new_angles
						self.wheel_power = new_power
						self.counter = new_counter
						self.robot_enabled = new_robot_enabled
						self.robot_mode = new_robot_mode
					
					self._broadcast_to_all({
						"type": "state",
						"angles": self.wheel_angles,
						"power": self.wheel_power,
						"counter": self.counter,
						"robot_enabled": self.robot_enabled,
						"robot_mode": self.robot_mode
					})
					
					if first_read and self.counter > 0:
						print(f"[NT] Connected to NetworkTables - broadcasting updates")
						first_read = False
						
				except Exception as e:
					pass
			
			time.sleep(0.05)

@app.route("/")
def serve_dashboard():
	"""Serve the dashboard - teleop or test mode"""
	mode = request.args.get("mode", "teleop")
	if mode == 'test':
		return render_template("test_dashboard.html")
	else:
		return render_template("teleop_dashboard.html")

@app.route("/history")
def history():
	"""Serve tuning history page"""
	return render_template("history.html")

if HAS_FLASK_SOCK:
	@sock.route("/ws")
	def websocket(ws):
		"""WebSocket endpoint for real-time dashboard updates"""
		dashboard.register_ws_client(ws)
		
		try:
			while True:
				try:
					message = ws.receive()
					if message is None:
						break
					
					data = json.loads(message)
					cmd = data.get("cmd")
					value = data.get("value")
					
					if not dashboard.table:
						ws.send(json.dumps({"type": "error", "message": "Not connected to NetworkTables"}))
						continue
					
					if cmd == "focus":
						dashboard.table.putString("focused_wheel", value)
					elif cmd == "power":
						dashboard.table.putNumber("wheel_power", float(value))
					elif cmd == "align":
						dashboard.table.putBoolean("align_command", True)
					elif cmd == "save_zero":
						dashboard.table.putBoolean("save_zero_command", True)
					elif cmd == "calibrate":
						wheel = value.get("wheel")
						angle = value.get("angle", 0)
						dashboard.table.putString("calibrate_wheel", wheel)
						dashboard.table.putNumber("calibrate_angle", angle)
					elif cmd == "set_wheel_angle":
						wheel = value.get("wheel")
						angle = value.get("angle", 0)
						dashboard.table.putString("set_wheel_angle_name", wheel)
						dashboard.table.putNumber("set_wheel_angle_value", angle)
					elif cmd == "autotune":
						# Autotune only in TEST mode
						robot_mode = dashboard.table.getString("robot_mode", "Unknown")
						if robot_mode == "Test":
							dashboard.table.putBoolean("autotune_command", True)
							print(f"[WS] autotune_command = true (TEST MODE)")
						else:
							ws.send(json.dumps({"type": "error", "message": "Autotune only allowed in TEST mode"}))
					elif cmd == "tuning_history":
						# Read tuning history from NetworkTables and send to client
						try:
							history_json_str = dashboard.table.getString("autotune_history_json", "[]")
							regression_json_str = dashboard.table.getString("autotune_regression_json", "{}")
							history = json.loads(history_json_str)
							regression = json.loads(regression_json_str)
							ws.send(json.dumps({
								"type": "tuning_history",
								"history": history,
								"regression": regression
							}))
						except Exception as e:
							ws.send(json.dumps({"type": "error", "message": f"Failed to load tuning history: {str(e)}"}))
					elif cmd == "clear_tuning":
						# Clear tuning history - only in TEST mode
						robot_mode = dashboard.table.getString("robot_mode", "Unknown")
						if robot_mode == "Test":
							try:
								dashboard.table.putString("autotune_history_json", "[]")
								dashboard.table.putString("autotune_regression_json", "{}")
								ws.send(json.dumps({"type": "success", "message": "Tuning history cleared"}))
							except Exception as e:
								ws.send(json.dumps({"type": "error", "message": f"Failed to clear history: {str(e)}"}))
						else:
							ws.send(json.dumps({"type": "error", "message": "Clear history only allowed in TEST mode"}))
				
				except json.JSONDecodeError:
					ws.send(json.dumps({"type": "error", "message": "Invalid JSON"}))
				except Exception as e:
					ws.send(json.dumps({"type": "error", "message": str(e)}))
		
		except Exception as e:
			print(f"[WS-ERR] WebSocket error: {e}")
		finally:
			dashboard.unregister_ws_client(ws)

# Create dashboard instance
dashboard = DashboardServer()

print("[WEB] Starting TELEOP Dashboard...")
dashboard.start_nt_listener()

if __name__ == "__main__":
	print("[WEB] Dashboard on http://localhost:5000")
	print("[WEB] Teleop: http://localhost:5000/")
	print("[WEB] Test: http://localhost:5000/?mode=test")
	print("[WEB] History: http://localhost:5000/history")
	app.run(host="0.0.0.0", port=5000, debug=True)
