"""Swerve drive odometry - dead reckoning via wheel encoders.

Coordinate system:
  - x, y in centimeters, field frame
  - heading in degrees (0-360), counterclockwise positive
  - heading = 0 means the robot's forward direction aligns with field +X axis

Robot frame (used internally):
  - +rx = robot forward
  - +ry = robot right
  - Wheel angle convention from existing code: 180 = forward, 90 = right,
    270 = left, 0 = backward

Expandability:
  - set_position(x, y)  -- call with camera/AprilTag fix
  - set_heading(deg)    -- call with IMU/gyro reading
  These are plain setters now; sensor fusion weighting can be added later.
"""
import math
from . import swerve_config as config


class SwerveOdometry:

	# REV EasySwerve (REV-21-3006) hardware constants
	DRIVE_GEAR_RATIO = 6.3				# motor rotations per wheel rotation
	WHEEL_DIAMETER_CM = 10.16			# 4 inch wheel
	WHEEL_CIRCUMFERENCE_CM = math.pi * 10.16

	def __init__(self, wheels_dict):
		self.wheels = wheels_dict

		# Robot pose - field coordinates
		self._x = 0.0
		self._y = 0.0
		self._heading = 0.0

		# Total scalar distance driven, for drive_for_distance
		self._total_distance_cm = 0.0

		# Heading delta computed last update() call (degrees, from wheel kinematics).
		# Used by IMU sensor fusion to compare against gyro rate.
		self._last_heading_delta = 0.0

		# Previous drive encoder positions (motor rotations)
		self._prev_positions = {}
		self._snapshot_encoders()

	def _snapshot_encoders(self):
		for name, wheel in self.wheels.items():
			self._prev_positions[name] = wheel.get_drive_position()

	# ------------------------------------------------------------------
	# Pose setters - call these to initialize or correct the pose
	# ------------------------------------------------------------------

	def set_position(self, x, y):
		"""Set robot position in centimeters (field frame)."""
		self._x = x
		self._y = y

	def set_heading(self, heading):
		"""Set robot heading in degrees (0-360)."""
		self._heading = heading % 360

	def reset(self):
		"""Zero out pose and distance counter, re-snapshot encoders."""
		self._x = 0.0
		self._y = 0.0
		self._heading = 0.0
		self._total_distance_cm = 0.0
		self._snapshot_encoders()

	# ------------------------------------------------------------------
	# Pose getters
	# ------------------------------------------------------------------

	def get_position(self):
		return (self._x, self._y)

	def get_x(self):
		return self._x

	def get_y(self):
		return self._y

	def get_heading(self):
		return self._heading

	def get_total_distance(self):
		return self._total_distance_cm

	def get_distance_traveled(self):
		return self._total_distance_cm

	def get_distance_meters(self):
		return self._total_distance_cm / 100.0

	def get_last_heading_delta(self):
		"""Heading change (degrees) computed from wheel kinematics in the last update() call."""
		return self._last_heading_delta

	# ------------------------------------------------------------------
	# Dead reckoning update - call every robot loop
	# ------------------------------------------------------------------

	def update(self):
		"""
		Integrate wheel encoder deltas into x, y, heading.

		For each wheel:
		  - Drive encoder delta / gear_ratio * circumference = distance this loop
		  - Wheel angle tells us the direction of that displacement in robot frame
		  - Average displacement across 4 wheels = robot translation
		  - Cross-product of wheel positions and displacements = heading change

		Returns average distance driven this loop (cm).
		"""
		# Half-distances from center to wheel in cm (rectangular robot).
		# Config positions are normalized (+-0.5); scale by actual trackwidth/wheelbase.
		half_x_cm = config.ROBOT_TRACKWIDTH_CM / 2.0   # left <-> right axis
		half_y_cm = config.ROBOT_WHEELBASE_CM  / 2.0   # front <-> rear axis

		# Capture heading before this update for the field rotation transform
		heading_before = self._heading

		sum_rx = 0.0
		sum_ry = 0.0
		rot_num = 0.0
		rot_den = 0.0
		total_abs_dist = 0.0

		for wheel_name, wheel in self.wheels.items():
			# Drive encoder delta -> wheel distance in cm
			cur = wheel.get_drive_position()
			delta_motor = cur - self._prev_positions[wheel_name]
			self._prev_positions[wheel_name] = cur

			dist_cm = (delta_motor / self.DRIVE_GEAR_RATIO) * self.WHEEL_CIRCUMFERENCE_CM
			total_abs_dist += abs(dist_cm)

			# Wheel angle -> robot-frame displacement vector
			# Angle convention: 180 = forward (+rx), 90 = right (+ry)
			# rx = cos(180 - angle), ry = sin(180 - angle)
			a = math.radians(180.0 - wheel.get_angle())
			rx = math.cos(a) * dist_cm
			ry = math.sin(a) * dist_cm

			sum_rx += rx
			sum_ry += ry

			# Wheel position in cm from robot center (x = left/right, y = front/rear)
			wx = config.WHEELS[wheel_name]["position"]["x"] * half_x_cm
			wy = config.WHEELS[wheel_name]["position"]["y"] * half_y_cm

			# Heading contribution: omega = sum(wx*ry - wy*rx) / sum(|r|^2)
			r2 = wx * wx + wy * wy
			if r2 > 0:
				rot_num += wx * ry - wy * rx
				rot_den += r2

		n = len(self.wheels)
		avg_rx = sum_rx / n
		avg_ry = sum_ry / n
		avg_dist = total_abs_dist / n

		# Update heading from wheel kinematics
		if rot_den > 0:
			omega_deg = math.degrees(rot_num / rot_den)
			self._last_heading_delta = omega_deg
			self._heading = (self._heading + omega_deg) % 360
		else:
			self._last_heading_delta = 0.0

		# Rotate robot-frame displacement into field frame using pre-update heading
		h = math.radians(heading_before)
		self._x += avg_rx * math.cos(h) - avg_ry * math.sin(h)
		self._y += avg_rx * math.sin(h) + avg_ry * math.cos(h)

		self._total_distance_cm += avg_dist

		return avg_dist
