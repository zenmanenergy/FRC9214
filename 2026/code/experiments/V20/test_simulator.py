#!/usr/bin/env python3
"""Test the waypoint navigator and simulator"""

import time
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from robot_simulator import RobotSimulator
from waypoint_navigator import WaypointNavigator


def test_simple_movement():
	"""Test basic robot movement"""
	print("=== TEST 1: Simple Movement ===")
	robot = RobotSimulator()
	
	# Move forward
	robot.set_wheel_angle("front_left", 0)
	robot.set_wheel_angle("front_right", 0)
	robot.set_wheel_angle("rear_left", 0)
	robot.set_wheel_angle("rear_right", 0)
	
	robot.set_all_wheels({"front_left": 0, "front_right": 0, "rear_left": 0, "rear_right": 0},
	                      {"front_left": 0.5, "front_right": 0.5, "rear_left": 0.5, "rear_right": 0.5})
	
	for i in range(100):
		robot.update()
		if i % 20 == 0:
			state = robot.get_state()
			print(f"  Step {i}: Pos=({state['odometry']['x']:.1f}, {state['odometry']['y']:.1f})")
	
	robot.stop_all()
	state = robot.get_state()
	print(f"Final: ({state['odometry']['x']:.1f}, {state['odometry']['y']:.1f})\n")


def test_waypoint_navigation():
	"""Test waypoint navigation system"""
	print("=== TEST 2: Waypoint Navigation ===")
	robot = RobotSimulator()
	nav = WaypointNavigator(robot)
	
	# Set waypoints: square pattern
	waypoints = [
		{"x": 100, "y": 0},
		{"x": 100, "y": 100},
		{"x": 0, "y": 100},
		{"x": 0, "y": 0}
	]
	
	nav.set_waypoints(waypoints)
	nav.start()
	
	# Simulate for ~30 seconds
	for i in range(1500):
		robot.update()
		nav.update()
		
		if i % 100 == 0:
			status = nav.get_status()
			state = robot.get_state()
			print(f"  Step {i}: {status['progress']}")
			print(f"           Pos=({state['odometry']['x']:.1f}, {state['odometry']['y']:.1f}) "
				  f"Heading={state['odometry']['heading']:.0f}°")
		
		if nav.is_finished():
			print(f"  Navigation complete at step {i}!")
			break
	
	print()


def test_rotation():
	"""Test in-place rotation"""
	print("=== TEST 3: In-Place Rotation ===")
	robot = RobotSimulator()
	
	# Rotate in place
	angles = {"front_left": 45, "front_right": 225, "rear_left": 225, "rear_right": 45}
	powers = {"front_left": 0.5, "front_right": 0.5, "rear_left": 0.5, "rear_right": 0.5}
	robot.set_all_wheels(angles, powers)
	
	for i in range(100):
		robot.update()
		if i % 20 == 0:
			state = robot.get_state()
			print(f"  Step {i}: Heading={state['odometry']['heading']:.0f}°")
	
	robot.stop_all()
	print()


def main():
	print("""
╔═══════════════════════════════════════════╗
║   Robot Simulator - Test Suite            ║
╚═══════════════════════════════════════════╝
	""")
	
	try:
		test_simple_movement()
		test_waypoint_navigation()
		test_rotation()
		
		print("✓ All tests completed!")
	except Exception as e:
		print(f"✗ Test failed: {e}")
		import traceback
		traceback.print_exc()


if __name__ == "__main__":
	main()
