"""Catmull-Rom spline implementation for smooth path following"""
import math


class CatmullRomSpline:
	"""Evaluate a Catmull-Rom spline through waypoints"""
	
	def __init__(self, waypoints):
		"""
		Initialize spline with waypoints.
		
		Args:
			waypoints: List of dicts with 'x', 'y', 'heading' keys (in cm and degrees)
		"""
		self.waypoints = waypoints
		self.num_segments = len(waypoints) - 1
		
		if self.num_segments < 1:
			raise ValueError("Need at least 2 waypoints for a spline")
	
	def evaluate(self, segment_idx, t):
		"""
		Evaluate spline position at parameter t within segment.
		
		Args:
			segment_idx: Segment index (0 to num_segments-1)
			t: Parameter within segment (0.0 to 1.0)
			
		Returns:
			dict with 'x', 'y' keys (in cm)
		"""
		if segment_idx < 0 or segment_idx >= self.num_segments:
			raise ValueError(f"Segment {segment_idx} out of range [0, {self.num_segments-1}]")
		
		if t < 0.0 or t > 1.0:
			raise ValueError(f"Parameter t={t} must be in [0.0, 1.0]")
		
		# For Catmull-Rom, we need 4 control points: p0, p1, p2, p3
		# The curve segment goes from p1 to p2
		# p0 is the point before p1, p3 is after p2
		
		# Clamp segment indices
		p0_idx = max(0, segment_idx - 1)
		p1_idx = segment_idx
		p2_idx = segment_idx + 1
		p3_idx = min(self.num_segments, segment_idx + 2)
		
		p0 = self.waypoints[p0_idx]
		p1 = self.waypoints[p1_idx]
		p2 = self.waypoints[p2_idx]
		p3 = self.waypoints[p3_idx]
		
		# Catmull-Rom matrix (alpha=0.5 uniform)
		t2 = t * t
		t3 = t2 * t
		
		x = 0.5 * (
			(2 * p1['x']) +
			(-p0['x'] + p2['x']) * t +
			(2 * p0['x'] - 5 * p1['x'] + 4 * p2['x'] - p3['x']) * t2 +
			(-p0['x'] + 3 * p1['x'] - 3 * p2['x'] + p3['x']) * t3
		)
		
		y = 0.5 * (
			(2 * p1['y']) +
			(-p0['y'] + p2['y']) * t +
			(2 * p0['y'] - 5 * p1['y'] + 4 * p2['y'] - p3['y']) * t2 +
			(-p0['y'] + 3 * p1['y'] - 3 * p2['y'] + p3['y']) * t3
		)
		
		return {'x': x, 'y': y}
	
	def evaluate_tangent(self, segment_idx, t):
		"""
		Evaluate spline tangent (derivative) at parameter t within segment.
		
		Args:
			segment_idx: Segment index
			t: Parameter within segment (0.0 to 1.0)
			
		Returns:
			dict with 'dx', 'dy' keys (tangent vector)
		"""
		if segment_idx < 0 or segment_idx >= self.num_segments:
			raise ValueError(f"Segment {segment_idx} out of range")
		
		p0_idx = max(0, segment_idx - 1)
		p1_idx = segment_idx
		p2_idx = segment_idx + 1
		p3_idx = min(self.num_segments, segment_idx + 2)
		
		p0 = self.waypoints[p0_idx]
		p1 = self.waypoints[p1_idx]
		p2 = self.waypoints[p2_idx]
		p3 = self.waypoints[p3_idx]
		
		# Derivative of Catmull-Rom
		t2 = t * t
		
		dx = 0.5 * (
			(-p0['x'] + p2['x']) +
			2 * (2 * p0['x'] - 5 * p1['x'] + 4 * p2['x'] - p3['x']) * t +
			3 * (-p0['x'] + 3 * p1['x'] - 3 * p2['x'] + p3['x']) * t2
		)
		
		dy = 0.5 * (
			(-p0['y'] + p2['y']) +
			2 * (2 * p0['y'] - 5 * p1['y'] + 4 * p2['y'] - p3['y']) * t +
			3 * (-p0['y'] + 3 * p1['y'] - 3 * p2['y'] + p3['y']) * t2
		)
		
		return {'dx': dx, 'dy': dy}
	
	def get_heading_from_tangent(self, dx, dy):
		"""
		Calculate heading angle from tangent vector.
		
		Args:
			dx, dy: Tangent components
			
		Returns:
			Heading in degrees (0-360)
		"""
		if abs(dx) < 1e-6 and abs(dy) < 1e-6:
			return 0.0
		
		heading = math.degrees(math.atan2(dy, dx))
		if heading < 0:
			heading += 360
		return heading
	
	def interpolate_heading(self, segment_idx, t):
		"""
		Interpolate desired heading from waypoint headings (not tangent direction).
		
		This allows the robot to follow a curved path while maintaining a specific heading.
		If both waypoints have heading=0, desired heading will be 0 throughout.
		
		Args:
			segment_idx: Which segment of the spline (0 to num_segments-1)
			t: Parameter within segment (0.0 to 1.0)
			
		Returns:
			Interpolated heading in degrees (0-360)
		"""
		# This segment interpolates from waypoint[segment_idx] to waypoint[segment_idx+1]
		heading1 = self.waypoints[segment_idx]['heading']
		heading2 = self.waypoints[segment_idx + 1]['heading']
		
		# Handle heading wrap-around (e.g., 350° to 10° should go +20° not -340°)
		diff = heading2 - heading1
		if diff > 180:
			diff -= 360
		elif diff < -180:
			diff += 360
		
		# Linear interpolation along segment
		interpolated = heading1 + diff * t
		if interpolated < 0:
			interpolated += 360
		elif interpolated >= 360:
			interpolated -= 360
		
		return interpolated
	
	def find_segment_for_distance(self, target_distance_cm):
		"""
		Find segment and parameter t for a given arc-length distance.
		Uses numerical integration (adaptive sampling).
		
		Args:
			target_distance_cm: Distance along curve from start (in cm)
			
		Returns:
			tuple (segment_idx, t, actual_distance_cm)
		"""
		if target_distance_cm <= 0:
			return (0, 0.0, 0.0)
		
		# Cap at total distance
		if target_distance_cm >= self.get_total_distance():
			return (self.num_segments - 1, 1.0, self.get_total_distance())
		
		accumulated_distance = 0.0
		samples_per_segment = 50  # Higher = more accurate arc-length
		
		for seg_idx in range(self.num_segments):
			for i in range(samples_per_segment):
				t0 = i / samples_per_segment
				t1 = (i + 1) / samples_per_segment
				
				p0 = self.evaluate(seg_idx, t0)
				p1 = self.evaluate(seg_idx, t1)
				
				segment_length = math.sqrt((p1['x'] - p0['x'])**2 + (p1['y'] - p0['y'])**2)
				
				if accumulated_distance + segment_length >= target_distance_cm:
					# Target is within this sub-segment
					# Linear interpolation between t0 and t1
					fraction = (target_distance_cm - accumulated_distance) / segment_length if segment_length > 0 else 0
					t_result = t0 + fraction * (t1 - t0)
					return (seg_idx, t_result, target_distance_cm)
				
				accumulated_distance += segment_length
		
		# End of spline reached
		return (self.num_segments - 1, 1.0, accumulated_distance)
	
	def get_total_distance(self):
		"""
		Calculate total arc-length distance of spline.
		
		Returns:
			Distance in cm
		"""
		total = 0.0
		samples_per_segment = 50
		
		for seg_idx in range(self.num_segments):
			for i in range(samples_per_segment):
				t0 = i / samples_per_segment
				t1 = (i + 1) / samples_per_segment
				
				p0 = self.evaluate(seg_idx, t0)
				p1 = self.evaluate(seg_idx, t1)
				
				segment_length = math.sqrt((p1['x'] - p0['x'])**2 + (p1['y'] - p0['y'])**2)
				total += segment_length
		
		return total
