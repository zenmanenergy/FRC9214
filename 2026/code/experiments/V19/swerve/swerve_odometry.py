"""Swerve drive odometry tracker using drive wheel encoders and swerve kinematics"""
import math
import time
from . import swerve_config as config


class SwerveOdometry:
	"""
	Tracks robot position and heading using swerve drive kinematics.
	Integrates drive wheel encoders and rotation wheel angles to calculate x, y, and heading.
	Designed to accept future updates from camera and IMU sensors.
	
	Wheel specifications:
	- 10.16 cm diameter wheels (4 inches)
	- REV NEO 1.1 with 42 CPR (counts per revolution)
	All measurements in metric (centimeters and meters)
	"""
	
	def __init__(self, wheels_dict, wheel_diameter_cm=10.16):
		"""
		Initialize odometry tracker.
		
		Args:
			wheels_dict: Dictionary of SwerveWheel objects keyed by name
			wheel_diameter_cm: Wheel diameter in centimeters (default 10.16 cm = 4 inches)
		"""
		self.wheels = wheels_dict
		self.wheel_diameter = wheel_diameter_cm
		
		# Robot position in centimeters
		self.x = 0.0
		self.y = 0.0
		self.heading = 0.0
		
		# Trust levels for position and heading (0.0 = don't use, 1.0 = fully trust)
		# Default to high encoder trust, low external source trust
		self.position_trust = 1.0
		self.heading_trust = 1.0
		
		# Decay rate configuration from swerve_config
		self.position_decay_per_meter = config.ODOMETRY_POSITION_DECAY_PER_METER
		self.heading_decay_per_meter = config.ODOMETRY_HEADING_DECAY_PER_METER
		self.position_decay_per_second = config.ODOMETRY_POSITION_DECAY_PER_SECOND
		self.heading_decay_per_second = config.ODOMETRY_HEADING_DECAY_PER_SECOND
		self.min_trust = config.ODOMETRY_MIN_TRUST
		
		# Track time for decay calculations
		self.last_update_time = 0.0
		
		# Track total distance in centimeters (legacy)
		self.total_distance = 0.0
		
		# Track previous positions to calculate delta
		self.prev_positions = {}
		self.prev_angles = {}
		for wheel_name in self.wheels.keys():
			self.prev_positions[wheel_name] = self.wheels[wheel_name].get_drive_position()
			self.prev_angles[wheel_name] = self.wheels[wheel_name].get_angle()
	
	def setXY(self, x=0.0, y=0.0):
		"""Set robot's x,y position in centimeters"""
		self.x = x
		self.y = y
	
	def getXY(self):
		"""Get robot's x,y position in centimeters as (x, y) tuple"""
		return (self.x, self.y)
	
	def setHeading(self, heading=0.0):
		"""Set robot's heading in degrees (0-360)"""
		self.heading = heading % 360
	
	def getHeading(self):
		"""Get robot's heading in degrees (0-360)"""
		return self.heading
	
	def reset(self):
		"""Reset position and distance tracking to zero"""
		self.x = 0.0
		self.y = 0.0
		self.heading = 0.0
		self.total_distance = 0.0
		for wheel_name in self.wheels.keys():
			self.prev_positions[wheel_name] = self.wheels[wheel_name].get_drive_position()
			self.prev_angles[wheel_name] = self.wheels[wheel_name].get_angle()
	
	def update(self):
		"""
		Update position and heading using swerve kinematics.
		Uses encoder data from all 4 wheels (distance + angle).
		Call this every robot loop.
		
		Returns the distance traveled in this loop (in centimeters).
		"""
		distances = []
		wheel_velocities = []
		
		# Get current data from each wheel
		for wheel_name, wheel in self.wheels.items():
			current_pos = wheel.get_drive_position()
			current_angle = wheel.get_angle()
			
			prev_pos = self.prev_positions[wheel_name]
			prev_angle = self.prev_angles[wheel_name]
			
			# Calculate delta position in rotations
			delta_rotations = current_pos - prev_pos
			
			# Convert rotations to centimeters
			wheel_distance = delta_rotations * math.pi * self.wheel_diameter
			distances.append(wheel_distance)
			
			# Get wheel position relative to robot center
			wheel_pos = config.WHEELS[wheel_name]["position"]
			
			# Convert wheel angle to radians
			angle_rad = math.radians(current_angle)
			
			# Velocity components in robot frame
			vx = wheel_distance * math.cos(angle_rad)
			vy = wheel_distance * math.sin(angle_rad)
			
			# Contribution to rotation (using wheel position from center)
			# Smaller wheels farther from center = more rotation
			if wheel_distance != 0:
				# Approximate rotation contribution based on wheel position
				rotation_contribution = (vy * wheel_pos["x"] - vx * wheel_pos["y"])
			else:
				rotation_contribution = 0
			
			wheel_velocities.append({
				"vx": vx,
				"vy": vy,
				"rotation": rotation_contribution
			})
			
			# Update previous values
			self.prev_positions[wheel_name] = current_pos
			self.prev_angles[wheel_name] = current_angle
		
		# Average velocity components across all wheels
		if wheel_velocities:
			avg_vx = sum([v["vx"] for v in wheel_velocities]) / len(wheel_velocities)
			avg_vy = sum([v["vy"] for v in wheel_velocities]) / len(wheel_velocities)
			avg_rotation = sum([v["rotation"] for v in wheel_velocities]) / len(wheel_velocities)
		else:
			avg_vx = 0.0
			avg_vy = 0.0
			avg_rotation = 0.0
		
		# Average distance across all wheels
		if distances:
			avg_distance = sum(distances) / len(distances)
		else:
			avg_distance = 0.0
		
		# Update position in robot frame
		if avg_distance != 0:
			# Convert to robot heading
			heading_rad = math.radians(self.heading)
			
			# Rotate velocity components to global frame
			global_vx = avg_vx * math.cos(heading_rad) - avg_vy * math.sin(heading_rad)
			global_vy = avg_vx * math.sin(heading_rad) + avg_vy * math.cos(heading_rad)
			
			# Update position
			self.x += global_vx
			self.y += global_vy
		
		# Update heading based on rotation contribution
		if avg_rotation != 0:
			# Convert rotation to heading change in degrees
			# Rotation is normalized by robot dimensions
			robot_wheelbase = 1.0
			heading_delta = math.degrees(avg_rotation / robot_wheelbase) if robot_wheelbase > 0 else 0
			self.heading = (self.heading + heading_delta) % 360
		
		# Accumulate total distance (legacy tracking)
		self.total_distance += abs(avg_distance)
		
		# Apply trust decay based on time and distance traveled
		current_time = time.time()
		if self.last_update_time > 0.0:
			elapsed_time = current_time - self.last_update_time
			distance_meters = abs(avg_distance) / 100.0  # convert cm to meters
			
			# Calculate decay from distance and time (whichever is worse)
			position_decay = max(
				distance_meters * self.position_decay_per_meter,
				elapsed_time * self.position_decay_per_second
			)
			heading_decay = max(
				distance_meters * self.heading_decay_per_meter,
				elapsed_time * self.heading_decay_per_second
			)
			
			# Apply decay and clamp at minimum trust
			self.position_trust = max(self.min_trust, self.position_trust - position_decay)
			self.heading_trust = max(self.min_trust, self.heading_trust - heading_decay)
		
		self.last_update_time = current_time
		
		return avg_distance
	
	def set_position(self, x, y):
		"""
		Set robot position in centimeters.
		Uses position_trust to blend with calculated position.
		Resets position trust to 1.0 (external verification received).
		
		Args:
			x: X position in centimeters
			y: Y position in centimeters
		"""
		if self.position_trust > 0.0:
			self.x = self.x * (1.0 - self.position_trust) + x * self.position_trust
			self.y = self.y * (1.0 - self.position_trust) + y * self.position_trust
		# Reset trust to 1.0 since external data provides fresh verification
		self.position_trust = 1.0
		self.last_update_time = time.time()
	
	def get_position(self):
		"""Get robot position as (x, y) tuple in centimeters"""
		return (self.x, self.y)
	
	def set_heading_degrees(self, heading):
		"""
		Set robot heading in degrees.
		Uses heading_trust to blend with calculated heading.
		Resets heading trust to 1.0 (external verification received).
		
		Args:
			heading: Heading in degrees (0-360)
		"""
		heading = heading % 360
		if self.heading_trust > 0.0:
			self.heading = (self.heading * (1.0 - self.heading_trust) + heading * self.heading_trust) % 360
		# Reset trust to 1.0 since external data provides fresh verification
		self.heading_trust = 1.0
		self.last_update_time = time.time()
	
	def get_heading_degrees(self):
		"""Get robot heading in degrees (0-360)"""
		return self.heading
	
	def set_position_trust(self, trust):
		"""
		Set position trust level (0.0 = ignore incoming, 1.0 = fully trust incoming).
		
		Args:
			trust: Trust level between 0.0 and 1.0
		"""
		self.position_trust = max(0.0, min(1.0, trust))
	
	def get_position_trust(self):
		"""Get current position trust level"""
		return self.position_trust
	
	def set_heading_trust(self, trust):
		"""
		Set heading trust level (0.0 = ignore incoming, 1.0 = fully trust incoming).
		
		Args:
			trust: Trust level between 0.0 and 1.0
		"""
		self.heading_trust = max(0.0, min(1.0, trust))
	
	def get_heading_trust(self):
		"""Get current heading trust level"""
		return self.heading_trust
	
	def get_total_distance(self):
		"""Get total distance traveled in centimeters"""
		return self.total_distance
	
	def get_distance_meters(self):
		"""Get total distance traveled in meters"""
		return self.total_distance / 100.0
