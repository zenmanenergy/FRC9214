#!/usr/bin/env python3
"""Standalone Robot Simulator - Updates NetworkTables without WPIlib"""

import ntcore
import time
import math
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(__file__))

from robot_simulator import RobotSimulator
from waypoint_navigator import WaypointNavigator


class SimulatorServer:
	"""Runs simulator and publishes to NetworkTables"""
	
	def __init__(self):
		self.robot = RobotSimulator()
		self.navigator = WaypointNavigator(self.robot)
		
		# Connect to NetworkTables
		print("[SIM] Connecting to NetworkTables...")
		inst = ntcore.NetworkTableInstance.getDefault()
		inst.startServer()  # Become a server
		
		self.table = inst.getTable("SmartDashboard")
		print("[SIM] Connected to SmartDashboard")
		
		self.running = True
		self.counter = 0
	
	def update_dashboard(self):
		"""Send robot state to NetworkTables"""
		state = self.robot.get_state()
		
		# Wheel angles
		self.table.putNumber("FR Angle", state["angles"]["front_right"])
		self.table.putNumber("RR Angle", state["angles"]["rear_right"])
		self.table.putNumber("RL Angle", state["angles"]["rear_left"])
		self.table.putNumber("FL Angle", state["angles"]["front_left"])
		
		# Wheel powers
		self.table.putNumber("FR Power", state["powers"]["front_right"])
		self.table.putNumber("RR Power", state["powers"]["rear_right"])
		self.table.putNumber("RL Power", state["powers"]["rear_left"])
		self.table.putNumber("FL Power", state["powers"]["front_left"])
		
		# Odometry
		self.table.putNumber("Odometry X", state["odometry"]["x"])
		self.table.putNumber("Odometry Y", state["odometry"]["y"])
		self.table.putNumber("Odometry Heading", state["odometry"]["heading"])
		self.table.putNumber("Distance Centimeters", state["odometry"]["distance"])
		
		# System
		self.table.putNumber("Counter", self.counter)
		self.table.putBoolean("Robot Enabled", True)
		self.table.putString("robot_mode", "Simulator")
		
		# Navigator status
		nav_status = self.navigator.get_status()
		self.table.putString("Navigator Status", nav_status["progress"])
	
	def receive_commands(self):
		"""Check for incoming commands from dashboard"""
		# Poll for navigation command
		try:
			# Read the navigation command flag
			nav_cmd = self.table.getBoolean("navigation_command", False)
			target_x = self.table.getNumber("navigation_target_x", 0)
			target_y = self.table.getNumber("navigation_target_y", 0)
			
			# Print every 100 cycles for debugging
			if self.counter % 100 == 0:
				print(f"[SIM-DBG] nav_cmd={nav_cmd}, target=({target_x:.1f}, {target_y:.1f})")
			
			if nav_cmd:
				print(f"[SIM] *** NAVIGATION COMMAND RECEIVED: Target ({target_x:.1f}, {target_y:.1f}) ***")
				
				# Set waypoint and start navigation
				self.navigator.set_waypoints([{"x": target_x, "y": target_y}])
				self.navigator.start()
				
				# Clear the command flag by writing false back
				self.table.putBoolean("navigation_command", False)
		except Exception as e:
			print(f"[SIM] Error receiving commands: {e}")
	
	def run(self):
		"""Main simulation loop"""
		print("[SIM] Starting simulation... (Press Ctrl+C to stop)")
		print("[SIM] Open dashboard at http://localhost:5000")
		
		loop_start = time.time()
		dt = 0.02  # 50Hz
		
		try:
			while self.running:
				loop_time = time.time()
				
				# Check for incoming commands from dashboard
				self.receive_commands()
				
				# Update robot and navigator
				self.robot.update()
				if self.navigator.is_active:
					self.navigator.update()
				
				# Publish to dashboard
				self.update_dashboard()
				self.counter += 1
				
				# Print status every 50 cycles
				if self.counter % 50 == 0:
					state = self.robot.get_state()
					print(f"[SIM] Cycle {self.counter}: Pos=({state['odometry']['x']:.1f}, "
						  f"{state['odometry']['y']:.1f}) Heading={state['odometry']['heading']:.0f}°")
				
				# Sleep to maintain frequency
				elapsed = time.time() - loop_time
				sleep_time = max(0, dt - elapsed)
				time.sleep(sleep_time)
		
		except KeyboardInterrupt:
			print("\n[SIM] Shutting down...")
		finally:
			self.running = False


def print_help():
	"""Print usage instructions"""
	print("""
╔════════════════════════════════════════════════════════════╗
║          ROBOT SIMULATOR - Usage Guide                     ║
╚════════════════════════════════════════════════════════════╝

1. START THE SIMULATOR:
   python robot_sim_server.py
   
   The simulator will:
   - Start a NetworkTables server
   - Update SmartDashboard values in real-time
   - Print position/heading updates every few seconds

2. START THE DASHBOARD (in another terminal):
   python dashboard/dashboard_server.py
   
   Then open: http://localhost:5000

3. WHAT THE SIMULATOR DOES:
   - Simulates wheel angles and power
   - Simulates robot odometry (position, heading)
   - Smoothly ramps angles and acceleration
   - Updates at 50Hz

4. TESTING CLICK-TO-NAVIGATE:
   - Open Field Map tab in dashboard
   - Click on the field to set a target
   - Click "Reset Odometry" to start at origin
   - Simulator will attempt to navigate to target

5. PROGRAMMATIC TEST:
   from robot_simulator import RobotSimulator
   from waypoint_navigator import WaypointNavigator
   
   robot = RobotSimulator()
   nav = WaypointNavigator(robot)
   nav.set_waypoints([
       {"x": 100, "y": 100},
       {"x": 200, "y": 200},
       {"x": 0, "y": 0}
   ])
   nav.start()
   
   # In your loop:
   robot.update()
   nav.update()

═════════════════════════════════════════════════════════════
	""")


if __name__ == "__main__":
	if "--help" in sys.argv or "-h" in sys.argv:
		print_help()
	else:
		server = SimulatorServer()
		server.run()
