import math


class gyro_odometry_fusion:
	"""
	Fuses odometry (encoder-based position) with gyro (heading reference).
	
	The challenge with pure odometry is that heading drifts due to:
	- Encoder quantization errors
	- Uneven wheel wear
	- Wheelbase measurement inaccuracies
	
	The gyro yaw is much more accurate for heading but can have noise and jumps.
	
	Solution: Use gyro yaw directly as heading (the source of truth for heading)
	and calculate x, y from encoder movements using the gyro-corrected heading.
	This gives us accurate position AND accurate heading.
	"""
	
	def __init__(self, odometry, gyro):
		"""
		Initialize sensor fusion.
		
		Args:
			odometry: Odometry instance (for position tracking via encoders)
			gyro: Gyro instance (for accurate heading via yaw)
		"""
		self.odometry = odometry
		self.gyro = gyro
		
		# Fused state
		self.x = 0.0
		self.y = 0.0
		self.fused_heading = 0.0  # Combined heading estimate from gyro and odometry (radians)
		
		# Track previous gyro heading to detect heading changes
		self.prev_gyro_heading = 0.0
		
		# For debugging
		self.odometry_gyro_heading_error = 0.0  # Difference between gyro and odometry heading
	
	def update(self):
		"""
		Update fused position and heading.
		Call this once per loop, after calling odometry.update() and before using the pose.
		
		The strategy:
		1. Get the gyro's yaw (most accurate heading)
		2. Calculate distance traveled from encoders
		3. Calculate heading change from encoders
		4. Use gyro yaw as the canonical heading (replace odometry's heading)
		5. Recalculate position with gyro-corrected heading
		"""
		# Get gyro heading in radians
		gyro_heading_deg = self.gyro.get_yaw()
		gyro_heading = math.radians(gyro_heading_deg)
		
		# Get raw odometry data
		odo_x, odo_y = self.odometry.get_position()
		odometry_heading = self.odometry.get_heading()
		
		# Get encoder distances for this update cycle
		# We need to recalculate from the current encoder counts
		left_distance = self.odometry._counts_to_centimeters(self.odometry.left_encoder.get())
		right_distance = self.odometry._counts_to_centimeters(self.odometry.right_encoder.get())
		
		avg_distance = (left_distance + right_distance) / 2.0
		odometry_delta_heading = (right_distance - left_distance) / self.odometry.WHEELBASE
		
		# The key insight: use gyro_heading, not encoder/odometry_heading
		# This corrects for encoder drift while maintaining position accuracy
		
		# Calculate the actual heading change from gyro
		delta_gyro_heading = gyro_heading - self.prev_gyro_heading
		
		# Normalize delta to [-pi, pi] to handle wraparound at ±180°
		delta_gyro_heading = math.atan2(math.sin(delta_gyro_heading), math.cos(delta_gyro_heading))
		
		# Update our canonical fused heading from the gyro
		self.fused_heading = gyro_heading
		
		# Recalculate position using gyro_heading for the movement direction
		# Use the midpoint heading for better accuracy
		mid_heading = self.fused_heading - (delta_gyro_heading / 2.0)
		
		# If moving significantly
		if abs(delta_gyro_heading) > 1e-6:
			# Arc method with gyro-corrected heading
			radius = avg_distance / delta_gyro_heading if abs(delta_gyro_heading) > 1e-10 else float('inf')
			old_heading = self.fused_heading - delta_gyro_heading
			
			center_x = self.x + radius * math.sin(old_heading)
			center_y = self.y - radius * math.cos(old_heading)
			
			self.x = center_x - radius * math.sin(self.fused_heading)
			self.y = center_y + radius * math.cos(self.fused_heading)
		else:
			# Straight line movement with gyro_heading
			self.x += avg_distance * math.cos(mid_heading)
			self.y += avg_distance * math.sin(mid_heading)
		
		# Track heading error for debugging: difference between gyro and odometry
		self.odometry_gyro_heading_error = math.degrees(self.fused_heading - odometry_heading)
		
		# Update for next iteration
		self.prev_gyro_heading = gyro_heading
	
	def get_position(self):
		"""Return (x, y) position in centimeters."""
		return (self.x, self.y)
	
	def get_heading(self):
		"""Return robot heading in radians from gyro-odometry fusion."""
		return self.fused_heading
	
	def get_heading_degrees(self):
		"""Return robot heading in degrees from gyro-odometry fusion."""
		return math.degrees(self.fused_heading)
	
	def get_pose(self):
		"""Return (x, y, heading) fused pose. Position in cm, heading in radians."""
		return (self.x, self.y, self.fused_heading)
	
	def reset(self):
		"""Reset fused position to origin and sync with current gyro_heading."""
		self.x = 0.0
		self.y = 0.0
		gyro_heading = math.radians(self.gyro.get_yaw())
		self.fused_heading = gyro_heading
		self.prev_gyro_heading = gyro_heading
		self.odometry_gyro_heading_error = 0.0
		
		# Also reset odometry
		self.odometry.reset()
	
	def set_position(self, x, y, heading=None):
		"""
		Manually set fused position.
		
		Args:
			x, y: Position in centimeters
			heading: Heading in radians (if None, uses current gyro_heading)
		"""
		self.x = x
		self.y = y
		if heading is not None:
			self.fused_heading = heading
		else:
			self.fused_heading = math.radians(self.gyro.get_yaw())
		
		self.prev_gyro_heading = self.fused_heading
	
	def print_debug_info(self):
		"""Print fused pose and comparison with raw odometry."""
		print("=== Gyro-Odometry Fusion Debug Info ===")
		print(f"Fused Position: ({self.x:.2f}, {self.y:.2f}) cm")
		print(f"Fused Heading: {self.get_heading_degrees():.2f}°")
		print(f"Gyro Heading: {self.gyro.get_yaw():.2f}°")
		
		# Compare with raw odometry
		odo_x, odo_y = self.odometry.get_position()
		odometry_heading = self.odometry.get_heading_degrees()
		print(f"\nRaw Odometry Position: ({odo_x:.2f}, {odo_y:.2f}) cm")
		print(f"Raw Odometry Heading: {odometry_heading:.2f}°")
		print(f"Heading Error (Gyro - Odometry): {self.odometry_gyro_heading_error:.2f}°")
		print("=====================================")
