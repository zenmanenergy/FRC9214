"""Swerve drive odometry tracker using drive wheel encoders"""
import math


class SwerveOdometry:
	"""
	Tracks robot distance traveled using average of all 4 drive wheel encoders.
	For swerve drive, we average all wheel distances (they should all be similar).
	
	Wheel specifications:
	- 4-inch diameter wheels
	- REV NEO 1.1 with 42 CPR (counts per revolution)
	"""
	
	def __init__(self, wheels_dict, wheel_diameter_inches=4.0):
		"""
		Initialize odometry tracker.
		
		Args:
			wheels_dict: Dictionary of SwerveWheel objects keyed by name
			wheel_diameter_inches: Wheel diameter in inches (default 4.0)
		"""
		self.wheels = wheels_dict
		self.wheel_diameter = wheel_diameter_inches
		
		# Track total distance in inches
		self.total_distance = 0.0
		
		# Track previous positions to calculate delta
		self.prev_positions = {}
		for wheel_name in self.wheels.keys():
			self.prev_positions[wheel_name] = self.wheels[wheel_name].get_drive_position()
	
	def reset(self):
		"""Reset distance tracking to zero"""
		self.total_distance = 0.0
		for wheel_name in self.wheels.keys():
			self.prev_positions[wheel_name] = self.wheels[wheel_name].get_drive_position()
	
	def update(self):
		"""
		Update distance traveled based on encoder movements.
		Call this every robot loop.
		Returns the distance traveled in this loop (in inches).
		"""
		distances = []
		
		# Get current position from each wheel
		for wheel_name, wheel in self.wheels.items():
			current_pos = wheel.get_drive_position()
			prev_pos = self.prev_positions[wheel_name]
			
			# Calculate delta position in rotations
			delta_rotations = current_pos - prev_pos
			
			# Convert rotations to inches
			# distance = rotations * pi * diameter
			wheel_distance = delta_rotations * math.pi * self.wheel_diameter
			distances.append(wheel_distance)
			
			# Update previous position
			self.prev_positions[wheel_name] = current_pos
		
		# Average distance across all wheels
		if distances:
			avg_distance = sum(distances) / len(distances)
		else:
			avg_distance = 0.0
		
		# Accumulate total distance (always positive)
		self.total_distance += abs(avg_distance)
		
		return avg_distance
	
	def get_total_distance(self):
		"""Get total distance traveled in inches"""
		return self.total_distance
	
	def get_distance_feet(self):
		"""Get total distance traveled in feet"""
		return self.total_distance / 12.0
	
	def get_distance_meters(self):
		"""Get total distance traveled in meters"""
		return self.total_distance * 0.0254
