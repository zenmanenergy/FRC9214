"""
Odometry integration example - shows how to connect real sensors to Navigation.
This file demonstrates how to set up encoder and gyro feedback for accurate positioning.
"""

import math
from wpilib import ADXRS450_Gyro, Encoder


class OdometryManager:
	"""Manages sensor-based odometry for robot positioning"""
	
	def __init__(self, drive, gyro=None, wheel_encoders=None):
		"""
		Initialize odometry with optional sensors.
		
		Args:
			drive: SwerveDrive instance
			gyro: ADXRS450_Gyro, NavX, Pigeon, or other gyro object with getAngle()
			wheel_encoders: Dict of {wheel_name: Encoder} for distance feedback
		"""
		self.drive = drive
		self.gyro = gyro
		self.encoders = wheel_encoders or {}
		
		# Calibration values
		self.WHEEL_DIAMETER_CM = 10.16  # 4 inch wheel
		self.ENCODER_CPR = 2048  # Falcon encoder counts per revolution
		self.DRIVE_GEAR_RATIO = 6.75  # Motor rotations per wheel rotation
		
		# Position tracking
		self.x = 0
		self.y = 0
		self.heading = 0
		self.last_heading = 0
		
		# Previous encoder values
		self.last_encoder_values = {}
		for wheel_name in self.encoders.keys():
			self.last_encoder_values[wheel_name] = 0
	
	def calibrate_gyro(self):
		"""Reset gyro to 0° (should be called when robot is stationary and aligned)"""
		if self.gyro:
			self.gyro.reset()
			print("[ODO] Gyro calibrated to 0°")
	
	def set_position(self, x, y, heading):
		"""Set known position (call at start of match or when position is known)"""
		self.x = x
		self.y = y
		self.heading = heading
		self.last_heading = heading
		print(f"[ODO] Position set to ({x:.1f}, {y:.1f}) heading {heading:.0f}°")
	
	def update(self):
		"""
		Update robot position from sensors.
		Call every robot cycle BEFORE using position values.
		
		Returns:
			(x, y, heading) - current position
		"""
		# Update heading from gyro
		old_heading = self.heading
		if self.gyro:
			try:
				gyro_angle = self.gyro.getAngle()
				self.heading = gyro_angle % 360
			except Exception as e:
				print(f"[ODO] Gyro error: {e}")
		
		# Calculate heading delta for forward/strafe decomposition
		dheading = self.heading - old_heading
		if dheading > 180:
			dheading -= 360
		elif dheading < -180:
			dheading += 360
		
		# Update position from wheel encoders
		if self.encoders:
			# Calculate average distance traveled by drive wheels
			distances = []
			for wheel_name, encoder in self.encoders.items():
				try:
					# Get encoder position
					counts = encoder.get()
					
					# Calculate wheel distance: counts / (CPR * gear_ratio) * wheel_circumference
					wheel_distance = (counts / (self.ENCODER_CPR * self.DRIVE_GEAR_RATIO)) * \
					                (math.pi * self.WHEEL_DIAMETER_CM)
					
					# Calculate delta since last update
					delta = wheel_distance - self.last_encoder_values.get(wheel_name, 0)
					distances.append(delta)
					
					# Update last value
					self.last_encoder_values[wheel_name] = wheel_distance
				except:
					pass
			
			# Average distance traveled (positive = forward)
			if distances:
				avg_distance = sum(distances) / len(distances)
				
				# Decompose into X and Y based on robot heading
				heading_rad = math.radians(self.heading)
				self.x += avg_distance * math.cos(heading_rad)
				self.y += avg_distance * math.sin(heading_rad)
		
		return self.x, self.y, self.heading
	
	def get_position(self):
		"""Get current position"""
		return self.x, self.y, self.heading


# ============================================================================
# EXAMPLE: How to integrate in robot.py
# ============================================================================

"""
In robot.py robotInit():

	# Initialize gyro (choose one based on your hardware)
	from wpilib import ADXRS450_Gyro, Rotation2d
	
	# ADXRS450 on SPI port
	gyro = ADXRS450_Gyro()
	
	# OR NavX (if you have one)
	# from navx import AHRS
	# gyro = AHRS(wpilib.SPI.Port.MXP)
	
	# Initialize encoder odometry
	self.odometry = OdometryManager(
		drive=self.drive,
		gyro=gyro,
		wheel_encoders={
			"front_left": Encoder(0, 1),
			"front_right": Encoder(2, 3),
			"rear_left": Encoder(4, 5),
			"rear_right": Encoder(6, 7),
		}
	)
	
	# Calibrate gyro (robot should be stationary, aligned)
	self.odometry.calibrate_gyro()
	
	# Set initial position (use a known starting location)
	self.odometry.set_position(x=0, y=0, heading=0)
	
	# Tell navigator to use gyro and calculate encoder deltas
	self.navigator.set_gyro(gyro)
	self.navigator.odometry_enabled = True

In teleopPeriodic():
	
	# Update odometry first
	self.odometry.update()
	
	# Navigator will now use gyro heading and encoder positions
	self.navigator.update()

"""
