"""
Dashboard server for real FRC robot with NetworkTables integration.
Provides web interface for monitoring and commanding swerve drive with X,Y navigation.
"""

import json
import asyncio
from flask import Flask, render_template
from flask_sock import Sock
from ntcore import NetworkTableInstance, EventFlags
import time


class RobotDashboardServer:
	"""Flask server bridging web dashboard to robot NetworkTables"""
	
	def __init__(self, host="0.0.0.0", port=5000, team_number=9214):
		self.app = Flask(__name__, template_folder="templates")
		self.sock = Sock(self.app)
		
		self.host = host
		self.port = port
		self.team_number = team_number
		
		# NetworkTables connection
		self.inst = NetworkTableInstance.getDefault()
		self.table = None
		self.connected = False
		
		# WebSocket clients
		self.ws_clients = []
		self.last_published_values = {}
		
		# Setup routes
		self.setup_routes()
		
	def connect_networktables(self):
		"""Connect to robot NetworkTables as client"""
		try:
			print(f"[DASH] Connecting to robot team {self.team_number}...")
			self.inst.startClient4(f"dashboard-{int(time.time())}")
			self.inst.setServerTeam(self.team_number)
			
			# Get SmartDashboard table
			self.table = self.inst.getTable("SmartDashboard")
			
			# Wait briefly for connection
			time.sleep(1)
			self.connected = True
			print("[DASH] Connected to NetworkTables!")
			return True
		except Exception as e:
			print(f"[DASH] Failed to connect: {e}")
			return False
	
	def setup_routes(self):
		"""Setup Flask routes and WebSocket handler"""
		
		@self.app.route("/")
		def serve_dashboard():
			return render_template("robot_dashboard.html")
		
		@self.sock.route("/ws")
		def websocket_handler(ws):
			"""Handle WebSocket connections from browser"""
			print("[WS] Client connected")
			self.ws_clients.append(ws)
			
			try:
				while True:
					try:
						message = ws.receive(timeout=1.0)
						if message:
							self.handle_websocket_message(ws, message)
					except:
						# Continue on timeout or other errors
						pass
					
					# Broadcast latest values to this client
					self._send_robot_state(ws)
					time.sleep(0.05)  # 20Hz update rate
					
			except Exception as e:
				print(f"[WS] Client error: {e}")
			finally:
				if ws in self.ws_clients:
					self.ws_clients.remove(ws)
				print("[WS] Client disconnected")
	
	def handle_websocket_message(self, ws, message):
		"""Handle incoming WebSocket message from browser"""
		try:
			data = json.loads(message)
			cmd = data.get("command")
			
			if cmd == "navigate_to":
				# Handle single target X,Y navigation command
				target_x = data.get("target_x", 0)
				target_y = data.get("target_y", 0)
				
				# Publish to NetworkTables
				self.table.putNumber("nav_target_x", target_x)
				self.table.putNumber("nav_target_y", target_y)
				self.table.putBoolean("nav_command", True)
				
				print(f"[WS] Navigation command: ({target_x:.1f}, {target_y:.1f}) cm")
				print(f"[WS] Published to NetworkTables: nav_target_x={target_x}, nav_target_y={target_y}, nav_command=true")
			
			elif cmd == "navigate_waypoints":
				# Handle multi-waypoint navigation command
				waypoints = data.get("waypoints", [])
				if not waypoints:
					print("[WS] ERROR: Empty waypoint list")
					return
				
				# Convert to JSON string for NetworkTables
				waypoints_json = json.dumps(waypoints)
				self.table.putString("nav_waypoints_json", waypoints_json)
				self.table.putBoolean("nav_waypoints_command", True)
				
				print(f"[WS] Waypoint navigation: {len(waypoints)} waypoints")
				for i, wp in enumerate(waypoints):
					print(f"[WS]   Waypoint {i+1}: ({wp.get('x', 0):.1f}, {wp.get('y', 0):.1f})")
				
		except json.JSONDecodeError:
			print(f"[WS] Invalid JSON: {message}")
		except Exception as e:
			print(f"[WS] Error handling message: {e}")
	
	def _send_robot_state(self, ws=None):
		"""
		Read robot state from NetworkTables and send to WebSocket.
		If ws is None, broadcast to all connected clients.
		"""
		if not self.connected or not self.table:
			return
		
		try:
			# Read robot state from SmartDashboard
			state = {
				# Wheel angles and powers
				"front_left_angle": self.table.getNumber("front_left_angle", 0),
				"front_left_power": self.table.getNumber("front_left_power", 0),
				"front_right_angle": self.table.getNumber("front_right_angle", 0),
				"front_right_power": self.table.getNumber("front_right_power", 0),
				"rear_left_angle": self.table.getNumber("rear_left_angle", 0),
				"rear_left_power": self.table.getNumber("rear_left_power", 0),
				"rear_right_angle": self.table.getNumber("rear_right_angle", 0),
				"rear_right_power": self.table.getNumber("rear_right_power", 0),
				
				# Robot position and heading
				"robot_x": self.table.getNumber("Robot X", 0),
				"robot_y": self.table.getNumber("Robot Y", 0),
				"robot_heading": self.table.getNumber("Robot Heading", 0),
				
				# Navigation state
				"nav_target_x": self.table.getNumber("Nav Target X", 0),
				"nav_target_y": self.table.getNumber("Nav Target Y", 0),
				"nav_active": self.table.getBoolean("Nav Active", False),
				"nav_distance": self.table.getNumber("Nav Distance", 0),
				
				# System state
				"robot_enabled": self.table.getBoolean("Robot Enabled", False),
				"robot_mode": self.table.getString("robot_mode", "Unknown"),
			}
			
			# Only send if values changed
			if state != self.last_published_values:
				self.last_published_values = state.copy()
				
				if ws:
					# Send to specific client
					try:
						ws.send(json.dumps(state))
					except:
						pass
				else:
					# Broadcast to all clients
					dead_clients = []
					for client in self.ws_clients:
						try:
							client.send(json.dumps(state))
						except:
							dead_clients.append(client)
					
					# Remove dead clients
					for client in dead_clients:
						if client in self.ws_clients:
							self.ws_clients.remove(client)
		
		except Exception as e:
			print(f"[DASH] Error sending state: {e}")
	
	def run(self, debug=True):
		"""Start the dashboard server"""
		if not self.connect_networktables():
			print("[DASH] ERROR: Could not connect to NetworkTables!")
			return False
		
		print(f"\n[DASH] Dashboard server starting on http://{self.host}:{self.port}")
		print("[DASH] Open browser at: http://localhost:5000")
		print("[DASH] Press Ctrl+C to stop\n")
		
		try:
			self.app.run(host=self.host, port=self.port, debug=debug, use_reloader=False)
		except KeyboardInterrupt:
			print("\n[DASH] Shutting down...")
		finally:
			self.inst.stopClient()


if __name__ == "__main__":
	import sys
	
	# Get team number from command line or use default
	team = 9214
	if len(sys.argv) > 1:
		try:
			team = int(sys.argv[1])
		except ValueError:
			print(f"Usage: python dashboard_server.py [team_number]")
			sys.exit(1)
	
	server = RobotDashboardServer(team_number=team)
	server.run(debug=True)
