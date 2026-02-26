import math
import wpilib


class Odometry:
	"""
	Tank drive odometry tracker using two REV through-bore encoders.
	Tracks robot position (x, y) and heading (theta) based on wheel movements.
	All measurements in centimeters and radians.
	
	REV Through-Bore Encoder specs:
	- Counts per revolution: 8192
	- Requires DIO (Digital I/O) channels for A and B signals
	"""
	
	def __init__(self, left_encoder, right_encoder, wheel_diameter_cm=15.24):
		"""
		Initialize odometry tracker.
		
		Args:
			left_encoder: REVPhotoelectricEncoder or similar (DIO A, DIO B)
			right_encoder: REVPhotoelectricEncoder or similar (DIO A, DIO B)
			wheel_diameter_cm: Wheel diameter in centimeters (default 15.24 cm = 6 inches)
		"""
		self.left_encoder = left_encoder
		self.right_encoder = right_encoder
		
		# Robot state (x, y position in centimeters, theta in radians)
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0  # Robot heading (radians, 0 = facing forward)
		
		# Encoder constants
		self.COUNTS_PER_REV = 2048  # REV through-bore encoder resolution (2048 cycles)
		self.WHEEL_DIAMETER = wheel_diameter_cm
		self.WHEEL_CIRCUMFERENCE = math.pi * self.WHEEL_DIAMETER
		
		# Track previous encoder counts to calculate delta
		self.prev_left_count = 0
		self.prev_right_count = 0
		
		# Robot geometry (wheelbase in centimeters)
		# This should be the distance between left and right wheels (center to center)
		self.WHEELBASE = 47.94  # TODO: Measure your actual wheelbase and update this
		
		# Reset encoders to zero
		self.reset()
	
	def reset(self):
		"""Reset position to origin and reset encoder counts."""
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.left_encoder.reset()
		self.right_encoder.reset()
		self.prev_left_count = 0
		self.prev_right_count = 0
	
	def _counts_to_centimeters(self, counts):
		"""Convert encoder counts to centimeters traveled."""
		revolutions = counts / self.COUNTS_PER_REV
		return revolutions * self.WHEEL_CIRCUMFERENCE
	
	def update(self):
		"""
		Update robot position based on encoder movements.
		Uses differential drive kinematics with arc radius calculation for accuracy.
		Call this every robot loop (usually 50 Hz).
		"""
		# Get current encoder counts (negate to convert direction)
		left_count = -self.left_encoder.get()
		right_count = -self.right_encoder.get()
		
		# Calculate change in counts
		delta_left_count = left_count - self.prev_left_count
		delta_right_count = right_count - self.prev_right_count
		
		# Convert count changes to distance changes (centimeters)
		left_distance = self._counts_to_centimeters(delta_left_count)
		right_distance = self._counts_to_centimeters(delta_right_count)
		
		# Update previous counts
		self.prev_left_count = left_count
		self.prev_right_count = right_count
		
		# Calculate robot movement using differential drive kinematics
		# Change in heading (radians) based on difference in wheel distances
		delta_theta = (right_distance - left_distance) / self.WHEELBASE
		
		# Average distance traveled (distance along the arc center)
		avg_distance = (left_distance + right_distance) / 2.0
		
		# If wheels moved significantly different amounts, use arc radius method
		# Otherwise, use straight-line approximation (faster, same result for small movements)
		if abs(delta_theta) > 1e-6:  # Not turning straight
			# Calculate radius of curvature (center of rotation)
			radius = avg_distance / delta_theta if abs(delta_theta) > 1e-10 else float('inf')
			
			# Midpoint heading before rotation
			old_theta = self.theta
			
			# Update heading
			self.theta += delta_theta
			self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
			
			# Calculate position change using arc geometry
			# The center of rotation is perpendicular to the initial heading
			center_x = self.x + radius * math.sin(old_theta)
			center_y = self.y - radius * math.cos(old_theta)
			
			# New position after rotating around the center
			self.x = center_x - radius * math.sin(self.theta)
			self.y = center_y + radius * math.cos(self.theta)
		else:  # Going straight (or nearly straight)
			# Use midpoint heading for better accuracy with finite time steps
			self.theta += delta_theta
			self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
			
			mid_theta = self.theta - (delta_theta / 2.0)
			self.x += avg_distance * math.cos(mid_theta)
			self.y += avg_distance * math.sin(mid_theta)
	
	def get_position(self):
		"""Return (x, y) position in centimeters."""
		return (self.x, self.y)
	
	def get_heading(self):
		"""Return robot heading in radians (0 = facing forward)."""
		return self.theta
	
	def get_heading_degrees(self):
		"""Return robot heading in degrees (0 = facing forward)."""
		return math.degrees(self.theta)
	
	def get_distance_traveled(self):
		"""Return total distance traveled in centimeters."""
		left_distance = self._counts_to_centimeters(self.left_encoder.get())
		right_distance = self._counts_to_centimeters(self.right_encoder.get())
		return (left_distance + right_distance) / 2.0
	
	def set_position(self, x, y, theta=0.0):
		"""Manually set robot position (in centimeters) and heading (in radians)."""
		self.x = x
		self.y = y
		self.theta = theta
	
	def get_pose(self):
		"""Return (x, y, theta) pose. Position in centimeters, heading in radians."""
		return (self.x, self.y, self.theta)
	
	def get_encoder_counts(self):
		"""Return raw encoder counts for debugging."""
		return (self.left_encoder.get(), self.right_encoder.get())
	
	def print_debug_info(self):
		"""Print position, heading, and raw encoder counts for calibration."""
		x, y = self.get_position()
		heading = self.get_heading_degrees()
		left_counts, right_counts = self.get_encoder_counts()
		print(f"Position: ({x:.2f}, {y:.2f}) cm, Heading: {heading:.2f}°")
		print(f"Encoder counts - Left: {left_counts}, Right: {right_counts}")