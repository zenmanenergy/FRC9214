"""Web Dashboard for Swerve Drive Calibration - WebSocket Version"""
from flask import Flask, render_template
import ntcore
import threading
import time
import json

try:
	from flask_sock import Sock
	HAS_FLASK_SOCK = True
except ImportError:
	HAS_FLASK_SOCK = False
	print("[ERROR] flask-sock not installed. Run: pip install flask-sock")

app = Flask(__name__)

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
		self.counter = 0
		self.robot_enabled = False
		self.running = False
		self.table = None
		
		# Autotune state
		self.autotune_complete = False
		self.autotune_status = "Idle"
		self.autotune_kp = 0.0
		self.autotune_ki = 0.0
		self.autotune_kd = 0.0
		self.autotune_wheel = "none"
		self.autotune_battery_voltage = 0.0
		
		# Tuning history (continuously synced from NT)
		self.tuning_history = []
		self.tuning_regression = None
		
		# WebSocket clients
		self.ws_clients = set()
		self.last_broadcast = {}  # Track what we last broadcast to avoid dupes
	
	def start_nt_listener(self):
		"""Start listening to NetworkTables"""
		try:
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
	
	def register_ws_client(self, ws):
		"""Register a WebSocket client"""
		self.ws_clients.add(ws)
		print(f"[WS] Client connected. Total clients: {len(self.ws_clients)}")
		# Send initial state to new client
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
				"counter": self.counter,
				"robot_enabled": self.robot_enabled,
				"autotune": {
					"complete": self.autotune_complete,
					"status": self.autotune_status,
					"wheel": self.autotune_wheel,
					"kp": self.autotune_kp,
					"ki": self.autotune_ki,
					"kd": self.autotune_kd,
					"battery_voltage": self.autotune_battery_voltage
				},
				"tuning_history": self.tuning_history,
				"tuning_regression": self.tuning_regression
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
		
		# Remove disconnected clients
		for ws in disconnected:
			self.ws_clients.discard(ws)
	
	def _update_values(self):
		"""Continuously read values from NetworkTables and broadcast changes"""
		first_read = True
		
		while self.running:
			if self.table:
				try:
					# Read all values
					new_angles = {
						"front_right": self.table.getNumber("FR Angle", 0),
						"rear_right": self.table.getNumber("RR Angle", 0),
						"rear_left": self.table.getNumber("RL Angle", 0),
						"front_left": self.table.getNumber("FL Angle", 0)
					}
					new_counter = self.table.getNumber("Counter", 0)
					new_robot_enabled = self.table.getBoolean("Robot Enabled", False)
					
					# Detect angle changes and broadcast
					if new_angles != self.wheel_angles or new_counter != self.counter or new_robot_enabled != self.robot_enabled:
						self.wheel_angles = new_angles
						self.counter = new_counter
						self.robot_enabled = new_robot_enabled
						
						self._broadcast_to_all({
							"type": "angles",
							"angles": self.wheel_angles,
							"counter": self.counter,
							"robot_enabled": self.robot_enabled
						})
					
					# Check autotune completion
					if self.table.getBoolean("autotune_complete", False):
						self.autotune_complete = True
						self.autotune_kp = self.table.getNumber("autotune_kp", 0.0)
						self.autotune_ki = self.table.getNumber("autotune_ki", 0.0)
						self.autotune_kd = self.table.getNumber("autotune_kd", 0.0)
						self.autotune_battery_voltage = self.table.getNumber("autotune_battery_voltage", 0.0)
						self.autotune_status = f"Complete! KP={self.autotune_kp:.6f}"
						
						self._broadcast_to_all({
							"type": "autotune_complete",
							"kp": self.autotune_kp,
							"ki": self.autotune_ki,
							"kd": self.autotune_kd,
							"battery_voltage": self.autotune_battery_voltage,
							"status": self.autotune_status
						})
						
						self.table.putBoolean("autotune_complete", False)
					
					# Check autotune progress
					wheel = self.table.getString("autotune_wheel", "none")
					if wheel != self.autotune_wheel:
						self.autotune_wheel = wheel
						if wheel != "none":
							self._broadcast_to_all({
								"type": "autotune_progress",
								"wheel": wheel
							})
					
					# Sync tuning history from NT
					try:
						history_json_str = self.table.getString("autotune_history_json", "[]")
						if history_json_str and history_json_str != "[]" and history_json_str != self.last_broadcast.get("history_json"):
							self.tuning_history = json.loads(history_json_str)
							self.last_broadcast["history_json"] = history_json_str
							
							regression_json_str = self.table.getString("autotune_regression_json", "{}")
							if regression_json_str and regression_json_str != "{}":
								self.tuning_regression = json.loads(regression_json_str)
							
							self._broadcast_to_all({
								"type": "tuning_history",
								"history": self.tuning_history,
								"regression": self.tuning_regression
							})
					except Exception as nt_sync_error:
						pass
					
					if first_read and self.counter > 0:
						print(f"[NT] Connected to NetworkTables - broadcasting updates")
						first_read = False
						
				except Exception as e:
					pass
			
			time.sleep(0.05)  # Update 20x per second

# Create dashboard instance
dashboard = DashboardServer()

# HTTP Routes
@app.route("/")
def index():
	return render_template("dashboard_ws.html")

@app.route("/history")
def history():
	return render_template("history.html")

@app.route("/calibration")
def calibration():
	return render_template("calibration.html")

# WebSocket endpoint
if HAS_FLASK_SOCK:
	print("[WEB] Registering WebSocket endpoint /ws")
	@sock.route("/ws")
	def websocket(ws):
		"""WebSocket endpoint for real-time dashboard updates"""
		print(f"[WS] New WebSocket connection from {ws}")
		dashboard.register_ws_client(ws)
		
		try:
			print("[WS] Entering message loop...")
			while True:
				try:
					message = ws.receive()
					print(f"[WS-RCV-RAW] Received: {message}")
					if message is None:
						print("[WS] Message is None, breaking...")
						break
					
					data = json.loads(message)
					cmd = data.get("cmd")
					value = data.get("value")
					
					print(f"[WS-RCV] Command: cmd='{cmd}' value='{value}'")
					
					if not dashboard.table:
						ws.send(json.dumps({"type": "error", "message": "Not connected to NetworkTables"}))
						continue
					
					# Handle commands
					if cmd == "focus":
						dashboard.table.putString("focused_wheel", value)
						print(f"[WS-SET] NetworkTables.focused_wheel = '{value}'")
					
					elif cmd == "power":
						dashboard.table.putNumber("wheel_power", float(value))
						print(f"[WS-SET] NetworkTables.wheel_power = {value}")
					
					elif cmd == "align":
						dashboard.table.putBoolean("align_command", True)
						print("[WS-SET] NetworkTables.align_command = true")
					
					elif cmd == "save_zero":
						dashboard.table.putBoolean("save_zero_command", True)
						print(f"[WS-SET] NetworkTables.save_zero_command = true for {value}")
					
					elif cmd == "calibrate":
						wheel = value.get("wheel")
						angle = value.get("angle", 0)
						dashboard.table.putString("calibrate_wheel", wheel)
						dashboard.table.putNumber("calibrate_angle", angle)
						print(f"[WS-SET] calibrate_wheel='{wheel}', calibrate_angle={angle}")
					
					elif cmd == "set_wheel_angle":
						wheel = value.get("wheel")
						angle = value.get("angle", 0)
						dashboard.table.putString("set_wheel_angle_name", wheel)
						dashboard.table.putNumber("set_wheel_angle_value", angle)
						print(f"[WS-SET] set_wheel_angle_name='{wheel}', set_wheel_angle_value={angle}")
					
					elif cmd == "set_wheels_direction":
						if isinstance(value, dict) and "angles" in value:
							# New format with focused_wheel
							dashboard.table.putString("set_wheels_direction", str(value["angles"]))
							dashboard.table.putString("focused_wheel_preset", value.get("focused_wheel", ""))
							print(f"[WS-SET] set_wheels_direction = {value['angles']}, focused_wheel = {value.get('focused_wheel', '')}")
						else:
							# Old format (backward compatibility) - all wheels same angle
							dashboard.table.putString("set_wheels_direction", str(value))
							dashboard.table.putString("focused_wheel_preset", "")
							print(f"[WS-SET] NetworkTables.set_wheels_direction = {value}")
					
					elif cmd == "autotune":
						dashboard.table.putBoolean("autotune_command", True)
						dashboard.autotune_complete = False
						dashboard.autotune_status = "Initializing..."
						dashboard.autotune_wheel = "none"
						print(f"[WS-SET] NetworkTables.autotune_command = true")
					
					elif cmd == "tuning_history_command":
						dashboard.table.putBoolean("tuning_history_command", True)
						print(f"[WS-SET] NetworkTables.tuning_history_command = true")
						# Immediately send tuning history
						ws.send(json.dumps({
							"type": "tuning_history",
							"history": dashboard.tuning_history,
							"regression": dashboard.tuning_regression
						}))
					
					elif cmd == "clear_tuning":
						dashboard.table.putBoolean("clear_tuning_command", True)
						dashboard.tuning_history = []
						dashboard.tuning_regression = None
						print(f"[WS-SET] NetworkTables.clear_tuning_command = true")
						ws.send(json.dumps({
							"type": "tuning_history",
							"history": [],
							"regression": None
						}))
					
					else:
						print(f"[WS] Unknown command: {cmd}")
					
					print(f"[WS-OK]\n")
					
				except json.JSONDecodeError:
					ws.send(json.dumps({"type": "error", "message": "Invalid JSON"}))
				except Exception as e:
					print(f"[WS-ERR] Command processing error: {e}")
					ws.send(json.dumps({"type": "error", "message": str(e)}))
		
		except Exception as e:
			print(f"[WS-ERR] WebSocket error: {e}")
		finally:
			dashboard.unregister_ws_client(ws)

# Start NetworkTables listener on module load
print("[WEB] Starting Web Dashboard...")
dashboard.start_nt_listener()

if __name__ == "__main__":
	print("[WEB] Starting Flask server on http://localhost:5000")
	app.run(host="0.0.0.0", port=5000, debug=True)
