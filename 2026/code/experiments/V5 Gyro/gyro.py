import wpilib
import math


class Gyro:
	"""
	NavX MXP IMU (Inertial Measurement Unit) wrapper for gyro and accelerometer data.
	Communicates via MXP (expansion port) over SPI or I2C.
	
	Focuses on gyroscope (angular velocity) and accelerometer (linear acceleration) data.
	Compass functionality is disabled due to unreliability.
	
	NavX MXP specs:
	- 3-axis gyroscope (degrees/sec)
	- 3-axis accelerometer (g-forces)
	- Sample rate: 100 Hz typical
	"""
	
	def __init__(self, port=wpilib.SPI.Port.kMXP):
		"""
		Initialize the NavX MXP IMU.
		
		Args:
			port: SPI port (default kMXP for MXP expansion port)
		"""
		try:
			self.ahrs = wpilib.AHRS(port)
			self.is_connected = True
		except RuntimeError as e:
			print(f"Error initializing NavX MXP: {e}")
			self.ahrs = None
			self.is_connected = False
		
		# Calibration offsets for gyro (stores bias during calibration)
		self.gyro_offset_x = 0.0
		self.gyro_offset_y = 0.0
		self.gyro_offset_z = 0.0
		
		# Calibration offsets for accelerometer
		self.accel_offset_x = 0.0
		self.accel_offset_y = 0.0
		self.accel_offset_z = 0.0
		
		# Initialize if connected
		if self.is_connected:
			self.calibrate()
	
	def calibrate(self):
		"""
		Calibrate gyro by zeroing the angle.
		Should be called when robot is stationary.
		"""
		if not self.is_connected:
			return
		
		self.ahrs.reset()
		print("NavX MXP calibrated")
	
	# ===== GYROSCOPE METHODS =====
	
	def get_yaw(self):
		"""
		Get yaw rotation (Z-axis) in degrees.
		Positive = counter-clockwise rotation when viewed from above.
		
		Returns:
			float: Yaw angle in degrees (-180 to 180)
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getYaw()
	
	def get_pitch(self):
		"""
		Get pitch rotation (Y-axis) in degrees.
		Positive = robot tilting forward (nose down).
		
		Returns:
			float: Pitch angle in degrees (-180 to 180)
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getPitch()
	
	def get_roll(self):
		"""
		Get roll rotation (X-axis) in degrees.
		Positive = right side tilting down.
		
		Returns:
			float: Roll angle in degrees (-180 to 180)
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getRoll()
	
	def get_yaw_rate(self):
		"""
		Get yaw rotation rate (angular velocity around Z-axis) in degrees/sec.
		Positive = counter-clockwise rotation when viewed from above.
		
		Returns:
			float: Yaw rate in degrees/sec
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getRate()
	
	def get_pitch_rate(self):
		"""
		Get pitch rotation rate (angular velocity around Y-axis) in degrees/sec.
		
		Returns:
			float: Pitch rate in degrees/sec
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getRawGyroY()
	
	def get_roll_rate(self):
		"""
		Get roll rotation rate (angular velocity around X-axis) in degrees/sec.
		
		Returns:
			float: Roll rate in degrees/sec
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getRawGyroX()
	
	def get_all_angles(self):
		"""
		Get all rotation angles at once.
		
		Returns:
			dict: {'yaw': float, 'pitch': float, 'roll': float} in degrees
		"""
		if not self.is_connected:
			return {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
		
		return {
			'yaw': self.get_yaw(),
			'pitch': self.get_pitch(),
			'roll': self.get_roll()
		}
	
	def get_all_rates(self):
		"""
		Get all angular velocity rates at once.
		
		Returns:
			dict: {'yaw_rate': float, 'pitch_rate': float, 'roll_rate': float} in degrees/sec
		"""
		if not self.is_connected:
			return {'yaw_rate': 0.0, 'pitch_rate': 0.0, 'roll_rate': 0.0}
		
		return {
			'yaw_rate': self.get_yaw_rate(),
			'pitch_rate': self.get_pitch_rate(),
			'roll_rate': self.get_roll_rate()
		}
	
	# ===== ACCELEROMETER METHODS =====
	
	def get_accel_x(self):
		"""
		Get X-axis acceleration (forward/backward) in g-forces.
		Positive = forward acceleration.
		
		Returns:
			float: Acceleration in g (1g = 9.81 m/s²)
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getRawAccelX()
	
	def get_accel_y(self):
		"""
		Get Y-axis acceleration (left/right) in g-forces.
		Positive = right-side acceleration.
		
		Returns:
			float: Acceleration in g (1g = 9.81 m/s²)
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getRawAccelY()
	
	def get_accel_z(self):
		"""
		Get Z-axis acceleration (up/down) in g-forces.
		Positive = upward acceleration.
		Normally ~1.0g when stationary (gravity).
		
		Returns:
			float: Acceleration in g (1g = 9.81 m/s²)
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getRawAccelZ()
	
	def get_all_accels(self):
		"""
		Get all acceleration axes at once.
		
		Returns:
			dict: {'x': float, 'y': float, 'z': float} in g-forces
		"""
		if not self.is_connected:
			return {'x': 0.0, 'y': 0.0, 'z': 0.0}
		
		return {
			'x': self.get_accel_x(),
			'y': self.get_accel_y(),
			'z': self.get_accel_z()
		}
	
	def get_linear_accel_x(self):
		"""
		Get linear acceleration (excluding gravity) along X-axis in g-forces.
		Removes the gravity component from the raw acceleration.
		
		Returns:
			float: Linear acceleration in g
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getWorldLinearAccelX()
	
	def get_linear_accel_y(self):
		"""
		Get linear acceleration (excluding gravity) along Y-axis in g-forces.
		
		Returns:
			float: Linear acceleration in g
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getWorldLinearAccelY()
	
	def get_linear_accel_z(self):
		"""
		Get linear acceleration (excluding gravity) along Z-axis in g-forces.
		
		Returns:
			float: Linear acceleration in g
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getWorldLinearAccelZ()
	
	def get_all_linear_accels(self):
		"""
		Get all linear acceleration axes (gravity-removed) at once.
		
		Returns:
			dict: {'x': float, 'y': float, 'z': float} in g-forces
		"""
		if not self.is_connected:
			return {'x': 0.0, 'y': 0.0, 'z': 0.0}
		
		return {
			'x': self.get_linear_accel_x(),
			'y': self.get_linear_accel_y(),
			'z': self.get_linear_accel_z()
		}
	
	# ===== UTILITY METHODS =====
	
	def get_temp(self):
		"""
		Get temperature of the NavX board in Celsius.
		
		Returns:
			float: Temperature in °C
		"""
		if not self.is_connected:
			return 0.0
		
		return self.ahrs.getTempC()
	
	def is_moving(self, accel_threshold=0.1):
		"""
		Determine if robot is moving based on linear acceleration.
		
		Args:
			accel_threshold: Minimum linear acceleration magnitude to consider "moving" (in g)
		
		Returns:
			bool: True if moving, False if stationary
		"""
		if not self.is_connected:
			return False
		
		accels = self.get_all_linear_accels()
		magnitude = math.sqrt(accels['x']**2 + accels['y']**2 + accels['z']**2)
		return magnitude > accel_threshold
	
	def is_tilted(self, tilt_threshold=5.0):
		"""
		Determine if robot is tilted based on roll and pitch.
		
		Args:
			tilt_threshold: Minimum tilt angle in degrees to consider "tilted"
		
		Returns:
			bool: True if tilted, False if level
		"""
		if not self.is_connected:
			return False
		
		angles = self.get_all_angles()
		tilt = math.sqrt(angles['pitch']**2 + angles['roll']**2)
		return tilt > tilt_threshold
	
	def reset_yaw(self):
		"""Reset yaw angle to zero (robot heading resets to forward)."""
		if self.is_connected:
			self.ahrs.reset()
	
	def set_yaw(self, angle_degrees):
		"""
		Set yaw angle to a specific value.
		Useful for syncing with vision system (e.g., AprilTag detection).
		
		Args:
			angle_degrees: Desired yaw angle in degrees
		"""
		if self.is_connected:
			self.ahrs.setAngleAdjustment(angle_degrees - self.get_yaw())
	
	def get_connection_status(self):
		"""
		Get NavX connection status.
		
		Returns:
			dict: Connection information
		"""
		if not self.is_connected:
			return {'connected': False, 'error': 'NavX not initialized'}
		
		return {
			'connected': True,
			'fw_version': self.ahrs.getFirmwareVersion(),
			'update_rate': self.ahrs.getActualUpdateRate(),
			'temp_c': self.get_temp()
		}
	
	def print_debug_info(self):
		"""Print debug information for troubleshooting."""
		if not self.is_connected:
			print("NavX MXP: NOT CONNECTED")
			return
		
		angles = self.get_all_angles()
		rates = self.get_all_rates()
		accels = self.get_all_accels()
		linear_accels = self.get_all_linear_accels()
		
		print("=== NavX MXP Debug Info ===")
		print(f"Angles (deg): Yaw={angles['yaw']:.2f}, Pitch={angles['pitch']:.2f}, Roll={angles['roll']:.2f}")
		print(f"Rates (deg/s): Yaw={rates['yaw_rate']:.2f}, Pitch={rates['pitch_rate']:.2f}, Roll={rates['roll_rate']:.2f}")
		print(f"Raw Accel (g): X={accels['x']:.3f}, Y={accels['y']:.3f}, Z={accels['z']:.3f}")
		print(f"Linear Accel (g): X={linear_accels['x']:.3f}, Y={linear_accels['y']:.3f}, Z={linear_accels['z']:.3f}")
		print(f"Temp: {self.get_temp():.1f}°C")
		print("=========================")
