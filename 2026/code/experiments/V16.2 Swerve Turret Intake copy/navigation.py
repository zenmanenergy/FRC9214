"""
Navigation module for autonomous X,Y coordinate-based movement.
Works with swerve drive and uses odometry for position tracking.
Supports both single targets and multi-waypoint navigation.
"""

import math
import time
from wpilib import SmartDashboard


class Navigation:
	"""Handles autonomous navigation to X,Y coordinates with waypoint support"""
	
	def __init__(self, swerve_drive):
		"""
		Initialize navigation system.
		
		Args:
			swerve_drive: SwerveDrive instance for movement control
		"""
		self.drive = swerve_drive
		
		# Position tracking
		self.x = 0  # Current X position (cm)
		self.y = 0  # Current Y position (cm)
		self.heading = 0  # Current heading (degrees)
		
		# Navigation state
		self.is_navigating = False
		self.waypoints = []  # List of {x, y} targets
		self.current_waypoint_index = 0
		self.target_x = 0
		self.target_y = 0
		self.start_time = None
		
		# Odometry sources
		self.use_gyro = False  # If True, use gyro heading instead of dead reckoning
		self.gyro = None  # NavX, Pigeon, or other gyro object
		self.encoders = None  # Wheel encoder data
		
		# Navigation parameters
		self.MAX_SPEED = 0.8  # Max swerve power
		self.ROTATION_SPEED = 0.5  # Max rotation power
		self.POSITION_TOLERANCE = 5  # cm
		self.HEADING_TOLERANCE = 5  # degrees
		self.TIMEOUT = 20  # seconds per waypoint
		
		# Odometry state
		self.last_gyro_heading = 0
		self.odometry_enabled = False
		
	def set_gyro(self, gyro_object):
		"""
		Set gyro sensor for heading feedback.
		
		Args:
			gyro_object: NavX, Pigeon, or other gyro with getAngle() method
		"""
		self.gyro = gyro_object
		self.use_gyro = True
		print("[NAV] Gyro enabled for heading tracking")
		
	def set_encoders(self, encoder_callback):
		"""
		Set encoder callback for position updates.
		
		Args:
			encoder_callback: Function that returns (delta_x_cm, delta_y_cm)
		"""
		self.encoders = encoder_callback
		self.odometry_enabled = True
		print("[NAV] Encoders enabled for odometry")
	
	def set_position(self, x, y, heading=0):
		"""Set current robot position (for calibration/initialization)"""
		self.x = x
		self.y = y
		self.heading = heading
		self.last_gyro_heading = heading
		SmartDashboard.putNumber("Robot X", self.x)
		SmartDashboard.putNumber("Robot Y", self.y)
		SmartDashboard.putNumber("Robot Heading", self.heading)
		print(f"[NAV] Position reset to ({x:.1f}, {y:.1f}) heading {heading:.0f}°")
		
	def update_odometry(self):
		"""
		Update position from sensors (encoders + gyro).
		Call this every robot cycle BEFORE update().
		"""
		if not self.odometry_enabled:
			return
		
		# Update heading from gyro if available
		if self.use_gyro and self.gyro:
			try:
				gyro_angle = self.gyro.getAngle()
				# Normalize to 0-360
				gyro_angle = gyro_angle % 360
				self.heading = gyro_angle
			except:
				pass  # Fall back to dead reckoning
		
		# Update position from encoders if available
		if self.encoders:
			try:
				delta_x, delta_y = self.encoders()
				self.x += delta_x
				self.y += delta_y
			except:
				pass  # Fall back to dead reckoning
		
		# Publish updated position
		SmartDashboard.putNumber("Robot X", self.x)
		SmartDashboard.putNumber("Robot Y", self.y)
		SmartDashboard.putNumber("Robot Heading", self.heading)
		
	def update_position(self, dx, dy, dheading):
		"""
		Update position based on dead reckoning (fallback if no sensors).
		
		Args:
			dx: Change in X (cm)
			dy: Change in Y (cm)
			dheading: Change in heading (degrees)
		"""
		if not self.odometry_enabled:
			# Only update if odometry is disabled
			self.x += dx
			self.y += dy
			self.heading += dheading
			
			# Normalize heading to 0-360
			while self.heading < 0:
				self.heading += 360
			while self.heading >= 360:
				self.heading -= 360
			
			# Publish to dashboard
			SmartDashboard.putNumber("Robot X", self.x)
			SmartDashboard.putNumber("Robot Y", self.y)
			SmartDashboard.putNumber("Robot Heading", self.heading)
		
	def navigate_to(self, target_x, target_y):
		"""
		Start navigation to single target coordinate.
		
		Args:
			target_x: Target X position (cm)
			target_y: Target Y position (cm)
		"""
		self.waypoints = [{"x": target_x, "y": target_y}]
		self.current_waypoint_index = 0
		self._start_waypoint_navigation()
		
	def navigate_waypoints(self, waypoint_list):
		"""
		Start navigation through multiple waypoints.
		
		Args:
			waypoint_list: List of dicts like [{"x": 100, "y": 200}, {"x": 300, "y": 400}]
		"""
		if not waypoint_list:
			print("[NAV] ERROR: Empty waypoint list")
			return
		
		self.waypoints = waypoint_list
		self.current_waypoint_index = 0
		self._start_waypoint_navigation()
		
	def _start_waypoint_navigation(self):
		"""Internal: Start navigation to current waypoint"""
		if self.current_waypoint_index >= len(self.waypoints):
			print("[NAV] ERROR: Waypoint index out of range")
			return
		
		waypoint = self.waypoints[self.current_waypoint_index]
		self.target_x = waypoint["x"]
		self.target_y = waypoint["y"]
		self.is_navigating = True
		self.start_time = time.time()
		
		total_waypoints = len(self.waypoints)
		current_num = self.current_waypoint_index + 1
		
		if total_waypoints == 1:
			print(f"[NAV] Starting navigation to ({self.target_x:.1f}, {self.target_y:.1f})")
		else:
			print(f"[NAV] Starting waypoint {current_num}/{total_waypoints}: ({self.target_x:.1f}, {self.target_y:.1f})")
		
		SmartDashboard.putBoolean("Nav Active", True)
		SmartDashboard.putNumber("Nav Target X", self.target_x)
		SmartDashboard.putNumber("Nav Target Y", self.target_y)
		SmartDashboard.putNumber("Nav Waypoint Index", self.current_waypoint_index)
		SmartDashboard.putNumber("Nav Waypoint Total", len(self.waypoints))
		
	def update(self):
		"""
		Update navigation - call every robot cycle.
		Drives swerve toward target position.
		"""
		if not self.is_navigating:
			return
		
		# Update odometry from sensors first
		self.update_odometry()
		
		# Check timeout
		elapsed = time.time() - self.start_time
		if elapsed > self.TIMEOUT:
			print(f"[NAV] Timeout after {elapsed:.1f}s at waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
			self._advance_waypoint()
			return
		
		# Calculate distance and angle to target
		dx_to_target = self.target_x - self.x
		dy_to_target = self.target_y - self.y
		distance = math.sqrt(dx_to_target**2 + dy_to_target**2)
		
		# Check if we reached the target
		if distance < self.POSITION_TOLERANCE:
			print(f"[NAV] Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} at ({self.x:.1f}, {self.y:.1f})")
			self._advance_waypoint()
			return
		
		# Calculate angle to target (0° = +X axis, 90° = +Y axis)
		angle_to_target = math.degrees(math.atan2(dy_to_target, dx_to_target))
		
		# Calculate required heading to face target
		heading_error = angle_to_target - self.heading
		# Normalize to -180 to 180
		while heading_error > 180:
			heading_error -= 360
		while heading_error < -180:
			heading_error += 360
		
		# Calculate forward/strafe speeds
		# Scale down as we get closer to target
		speed_scale = min(1.0, distance / 100.0)  # Full speed at >100cm
		speed_scale = max(0.2, speed_scale)  # Minimum 0.2 speed
		
		# Convert target angle to swerve forward/strafe
		# Forward is in the direction we want to go
		forward = math.cos(math.radians(angle_to_target - self.heading)) * speed_scale * self.MAX_SPEED
		strafe = math.sin(math.radians(angle_to_target - self.heading)) * speed_scale * self.MAX_SPEED
		
		# Rotation to face target
		rotation = 0
		if abs(heading_error) > self.HEADING_TOLERANCE:
			rotation = max(-self.ROTATION_SPEED, min(self.ROTATION_SPEED, heading_error / 90.0))
		
		# Execute swerve command
		self.drive.drive_swerve(forward, strafe, rotation)
		
		# Publish debug info
		SmartDashboard.putNumber("Nav Distance", distance)
		SmartDashboard.putNumber("Nav Angle Error", heading_error)
		SmartDashboard.putNumber("Nav Forward", forward)
		SmartDashboard.putNumber("Nav Strafe", strafe)
		
	def _advance_waypoint(self):
		"""Internal: Move to next waypoint or stop if done"""
		self.current_waypoint_index += 1
		
		if self.current_waypoint_index >= len(self.waypoints):
			# All waypoints complete
			self.stop()
		else:
			# Start next waypoint
			self._start_waypoint_navigation()
		
	def stop(self):
		"""Stop navigation"""
		self.is_navigating = False
		self.drive.stop_all()
		SmartDashboard.putBoolean("Nav Active", False)
		print("[NAV] Navigation stopped")
		
	def is_active(self):
		"""Check if navigation is currently active"""
		return self.is_navigating
		
	def get_status(self):
		"""Get navigation status as dictionary"""
		return {
			"active": self.is_navigating,
			"x": self.x,
			"y": self.y,
			"heading": self.heading,
			"target_x": self.target_x,
			"target_y": self.target_y,
			"waypoint_index": self.current_waypoint_index,
			"waypoint_total": len(self.waypoints),
		}
