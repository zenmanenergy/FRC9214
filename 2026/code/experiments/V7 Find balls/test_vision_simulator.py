"""
Test script for BallClusterVision with RoboRio simulation
Simulates NetworkTables and provides synthetic camera frames with yellow balls
"""

import cv2
import numpy as np
import math
import threading
import time

# Mock NetworkTables for testing without RoboRio
class MockNetworkTable:
	def __init__(self, name):
		self.name = name
		self.data = {}
	
	def putNumber(self, key, value):
		self.data[key] = value
		print(f"[NT] {self.name}/{key} = {value}")
	
	def putBoolean(self, key, value):
		self.data[key] = value
		print(f"[NT] {self.name}/{key} = {value}")
	
	def getNumber(self, key, default=None):
		return self.data.get(key, default)

class MockNetworkTables:
	tables = {}
	initialized = False
	
	@staticmethod
	def initialize(server=None, team=None):
		MockNetworkTables.initialized = True
		print(f"[MOCK] NetworkTables initialized (server={server})")
	
	@staticmethod
	def getTable(name):
		if name not in MockNetworkTables.tables:
			MockNetworkTables.tables[name] = MockNetworkTable(name)
		return MockNetworkTables.tables[name]

# Patch the real networktables with mock
import sys
sys.modules['networktables'] = type(sys)('networktables')
sys.modules['networktables'].NetworkTables = MockNetworkTables

# Now import the vision class
from ball_cluster_vision import BallClusterVision

class RobotSimulator:
	"""Simulates robot pose moving around the field"""
	def __init__(self, vision_system):
		self.vision = vision_system
		self.x = 0.0
		self.y = 0.0
		self.heading = 0.0
		self.time = 0.0
	
	def update(self, dt=0.033):  # ~30 FPS
		"""Update robot pose (simple circular motion)"""
		self.time += dt
		
		# Circular motion: move in a circle
		radius = 2.0
		angular_velocity = 0.5  # rad/s
		
		angle = angular_velocity * self.time
		self.x = radius * math.cos(angle)
		self.y = radius * math.sin(angle)
		self.heading = angle + math.pi / 2
		
		# Update NetworkTables
		nt = self.vision.nt
		nt.putNumber("robot/x", self.x)
		nt.putNumber("robot/y", self.y)
		nt.putNumber("robot/heading", self.heading)

def create_test_frame_with_balls(width=640, height=480, num_balls=10, seed=None):
	"""Create a synthetic test frame with yellow balls"""
	if seed is not None:
		np.random.seed(seed)
	
	frame = np.ones((height, width, 3), dtype=np.uint8) * 100  # Gray background
	
	# Add some variation
	frame = cv2.GaussianBlur(frame, (5, 5), 0)
	
	# Add random noise
	noise = np.random.randint(-10, 10, frame.shape)
	frame = np.clip(frame.astype(int) + noise, 0, 255).astype(np.uint8)
	
	# Draw yellow balls (in BGR: yellow = (0, 255, 255))
	for i in range(num_balls):
		x = np.random.randint(50, width - 50)
		y = np.random.randint(50, height - 50)
		radius = np.random.randint(15, 40)
		
		# Draw filled circle
		cv2.circle(frame, (x, y), radius, (0, 255, 255), -1)
		# Add some shading
		cv2.circle(frame, (x, y), radius, (0, 200, 200), 2)
	
	return frame

def run_simulation(duration=10.0, num_frames=None):
	"""Run the vision system simulator"""
	print("=" * 60)
	print("BALL CLUSTER VISION SIMULATOR - WITHOUT ROBORIO")
	print("=" * 60)
	print()
	
	# Try to initialize vision system
	print("[INIT] Initializing BallClusterVision...")
	try:
		vision = BallClusterVision(server_ip="localhost", enable_debug=True)
		print("[OK] BallClusterVision initialized successfully!")
	except Exception as e:
		print(f"[ERROR] Failed to initialize vision: {e}")
		return False
	
	# Initialize robot simulator
	print("[INIT] Initializing RobotSimulator...")
	robot = RobotSimulator(vision)
	print("[OK] RobotSimulator initialized!")
	print()
	
	# Main loop
	frame_count = 0
	start_time = time.time()
	fps_clock = time.time()
	fps_count = 0
	
	print("[SIMULATION] Starting main loop...")
	print("-" * 60)
	
	try:
		while True:
			current_time = time.time() - start_time
			if current_time > duration:
				break
			
			if num_frames is not None and frame_count >= num_frames:
				break
			
			# Update robot pose
			robot.update(dt=0.033)
			
			# Create test frames with yellow balls
			left_frame = create_test_frame_with_balls(num_balls=8, seed=frame_count*2)
			right_frame = create_test_frame_with_balls(num_balls=8, seed=frame_count*2+1)
			
			# Run vision update
			print(f"\n[FRAME {frame_count}] Robot: ({robot.x:.2f}, {robot.y:.2f}) Head: {math.degrees(robot.heading):.1f}°")
			
			try:
				vision.update(left_frame, right_frame)
				print(f"[OK] Vision update successful")
			except Exception as e:
				print(f"[ERROR] Vision update failed: {e}")
				import traceback
				traceback.print_exc()
				return False
			
			# Display debug info (non-blocking for terminal)
			print(f"  Left Balls: {vision.debug_stats['left_balls']}")
			print(f"  Right Balls: {vision.debug_stats['right_balls']}")
			print(f"  Stereo Matches: {vision.debug_stats['stereo_matches']}")
			print(f"  Valid Points: {vision.debug_stats['valid_points']}")
			print(f"  Clusters: {vision.debug_stats['clusters']}")
			print(f"  Largest Cluster: {vision.debug_stats['largest_cluster_size']}")
			print(f"  Locked Confidence: {vision.debug_stats['locked_confidence']}")
			
			frame_count += 1
			fps_count += 1
			
			# Calculate FPS
			elapsed = time.time() - fps_clock
			if elapsed > 1.0:
				fps = fps_count / elapsed
				print(f"\n>>> FPS: {fps:.1f}")
				fps_count = 0
				fps_clock = time.time()
			
			# Small delay to avoid maxing out CPU
			time.sleep(0.01)
	
	except KeyboardInterrupt:
		print("\n[INTERRUPT] User interrupted simulation")
	
	except Exception as e:
		print(f"\n[FATAL ERROR] Unexpected error: {e}")
		import traceback
		traceback.print_exc()
		return False
	
	print()
	print("=" * 60)
	print(f"SIMULATION COMPLETE - Processed {frame_count} frames")
	print("=" * 60)
	
	return True

if __name__ == "__main__":
	print("\n")
	success = run_simulation(duration=10.0, num_frames=30)
	
	if success:
		print("\n✓ TEST PASSED - No errors detected!")
	else:
		print("\n✗ TEST FAILED - See errors above")
	
	print("\nPress Ctrl+C to exit...")
	try:
		while True:
			time.sleep(0.1)
	except KeyboardInterrupt:
		print("Exiting.")
