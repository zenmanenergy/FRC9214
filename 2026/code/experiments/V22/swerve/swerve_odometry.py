"""Swerve drive odometry - dead reckoning via wheel encoders.

Coordinate system:
  - x, y in centimeters, field frame
  - heading in degrees (0-360), counterclockwise positive
  - heading = 0 means the robot's forward direction aligns with field +X axis

Robot frame (used internally):
  - +rx = robot forward
  - +ry = robot right
  - Wheel angle convention: 180° = forward, 90° = right, 270° = left, 0° = backward

Sensor Fusion:
  - Use set_position(x, y) for camera/AprilTag corrections
  - Use set_heading(deg) for IMU fusion
  - Call imu.fuse_heading(odometry) for automatic complementary filtering

Example:
    odom = SwerveOdometry(wheels_dict)
    while True:
        odom.update()  # Update from wheel encoders
        imu.fuse_heading(odom)  # Blend IMU into heading
        x, y = odom.get_position()
        print(f'Position: {x:.1f}, {y:.1f} cm @ {odom.get_heading():.1f}°')
"""

from typing import Dict, Tuple
import math
from . import swerve_config as config


class SwerveOdometry:

	# REV EasySwerve (REV-21-3006) hardware constants
	DRIVE_GEAR_RATIO = 2.1				# motor rotations per wheel rotation (6.3 / 3 correction factor)
	WHEEL_DIAMETER_CM = 10.16			# 4 inch wheel
	WHEEL_CIRCUMFERENCE_CM = math.pi * 10.16

	def __init__(self, wheels_dict: Dict) -> None:
		"""Initialize odometry with reference to wheel objects.
		
		Args:
			wheels_dict: Dictionary of SwerveWheel objects keyed by wheel name
		"""
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

		# Odometry/IMU calibration corrections (see docs/plans/odometry_imu_calibration_plan.md).
		# Defaults are neutral (scale=1.0, no backlash) so behavior is unchanged until calibrated.
		self._wheel_scale_curve = {name: {"m": 0.0, "b": 1.0} for name in self.wheels}
		self._prev_power_sign = {name: 0 for name in self.wheels}
		self._rotation_scale_factor = 1.0
		self._axis_backlash_cm = {"forward_axis": 0.0, "strafe_axis": 0.0, "diagonal_axis": 0.0}

	def _snapshot_encoders(self):
		for name, wheel in self.wheels.items():
			self._prev_positions[name] = wheel.get_drive_position()

	# ------------------------------------------------------------------
	# Pose setters - call these to initialize or correct the pose
	# ------------------------------------------------------------------

	def set_position(self, x: float, y: float) -> None:
		"""Set robot position in centimeters (field frame)."""
		self._x = x
		self._y = y

	def set_heading(self, heading: float) -> None:
		"""Set robot heading in degrees (0-360)."""
		self._heading = heading % 360

	def reset(self) -> None:
		"""Zero out pose and distance counter, re-snapshot encoders."""
		self._x = 0.0
		self._y = 0.0
		self._heading = 0.0
		self._total_distance_cm = 0.0
		self._snapshot_encoders()

	# ------------------------------------------------------------------
	# Odometry/IMU calibration corrections
	# ------------------------------------------------------------------

	def set_wheel_scale_curve(self, wheel_name: str, m: float, b: float) -> None:
		"""Set the linear speed(0-1) -> distance-scale-factor curve for one wheel."""
		if wheel_name in self._wheel_scale_curve:
			self._wheel_scale_curve[wheel_name] = {"m": m, "b": b}

	def get_wheel_scale(self, wheel_name: str, speed_frac: float) -> float:
		"""Evaluate a wheel's distance-scale curve at the given speed fraction (0-1)."""
		curve = self._wheel_scale_curve.get(wheel_name, {"m": 0.0, "b": 1.0})
		return curve["m"] * speed_frac + curve["b"]

	def get_average_wheel_scale(self, speed_frac: float) -> float:
		"""Average distance-scale factor across all wheels at the given speed fraction."""
		if not self.wheels:
			return 1.0
		scales = [self.get_wheel_scale(name, speed_frac) for name in self.wheels]
		return sum(scales) / len(scales)

	def set_axis_backlash(self, axis: str, backlash_cm: float) -> None:
		"""Set the distance (cm) added once whenever a wheel on this axis reverses direction."""
		if axis in self._axis_backlash_cm:
			self._axis_backlash_cm[axis] = backlash_cm

	def set_rotation_scale_factor(self, factor: float) -> None:
		"""Set the correction factor applied to wheel-kinematics rotation (N-spin calibration)."""
		self._rotation_scale_factor = factor

	def load_calibration(self, calibration) -> None:
		"""Load persisted odometry corrections from an EncoderCalibration instance."""
		rotation_cal = calibration.get_rotation_calibration()
		self.set_rotation_scale_factor(rotation_cal.get("odometry_rotation_scale_factor", 1.0))

		for wheel_name in self.wheels:
			curve = calibration.get_wheel_scale_curve(wheel_name)
			self.set_wheel_scale_curve(wheel_name, curve.get("m", 0.0), curve.get("b", 1.0))

		backlash = calibration.get_reversal_backlash()
		# Backlash is stored per-axis; applied identically to whichever wheel reverses
		# since all 4 wheels share the same steer angle during pure translation.
		for axis, value in backlash.items():
			self.set_axis_backlash(axis, value)

	# ------------------------------------------------------------------
	# Pose getters
	# ------------------------------------------------------------------

	def get_position(self) -> Tuple[float, float]:
		"""Get current position as (x, y) in centimeters."""
		return (self._x, self._y)
	
	def get_x(self) -> float:
		"""Get x position in centimeters."""
		return self._x
	
	def get_y(self) -> float:
		"""Get y position in centimeters."""
		return self._y
	
	def get_heading(self) -> float:
		"""Get robot heading in degrees (0-360)."""
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

	def update(self) -> float:
		"""Update pose from wheel encoder deltas.
		
		Calls wheel.get_drive_position() to compute distances traveled,
		integrates into x/y/heading using kinematic model.
		
		Returns:
			Average distance driven this update cycle in centimeters
		"""
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

			# Speed-dependent per-wheel distance-scale correction from translation calibration
			# (see docs/plans/odometry_imu_calibration_plan.md section 6.5).
			speed_frac = abs(wheel.get_drive_power())
			curve = self._wheel_scale_curve.get(wheel_name)
			if curve is not None:
				scale = curve["m"] * speed_frac + curve["b"]
				if scale > 0:
					dist_cm *= scale

			# Reversal backlash: add a one-time distance correction whenever this
			# wheel's commanded drive direction flips sign (see section 6.2).
			power = wheel.get_drive_power()
			sign = 1 if power > 1e-3 else (-1 if power < -1e-3 else 0)
			prev_sign = self._prev_power_sign.get(wheel_name, 0)
			if sign != 0 and prev_sign != 0 and sign != prev_sign:
				axis = self._axis_for_angle(wheel.get_angle())
				backlash_cm = self._axis_backlash_cm.get(axis, 0.0)
				dist_cm += backlash_cm * sign
			if sign != 0:
				self._prev_power_sign[wheel_name] = sign

			total_abs_dist += abs(dist_cm)

			# Wheel angle -> robot-frame displacement vector
			# Hardware convention: 180 = forward (+rx), 270 = right (+ry)
			# rx = cos(angle - 180), ry = sin(angle - 180)
			a = math.radians(wheel.get_angle() - 180.0)
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

		# Update heading from wheel kinematics (will be fused with IMU)
		if rot_den > 0:
			omega_deg = math.degrees(rot_num / rot_den) * self._rotation_scale_factor
			self._last_heading_delta = omega_deg
			self._heading = (self._heading + omega_deg) % 360
		else:
			self._last_heading_delta = 0.0

		# Rotate robot-frame displacement into field frame using pre-update heading
		# The robot moved avg_rx forward and avg_ry to the right (in robot frame).
		# We need to rotate this displacement vector to field coordinates based on heading.
		# Using 2D rotation matrix where h is the robot's heading angle:
		#   field_x = robot_right * cos(h) - robot_forward * sin(h)
		#   field_y = robot_right * sin(h) + robot_forward * cos(h)
		# This ensures forward motion always increases field_y and right motion increases field_x
		# when heading = 0, and rotates correctly as the robot turns.
		h = math.radians(heading_before)
		self._x += avg_ry * math.cos(h) - avg_rx * math.sin(h)
		self._y += -1*(avg_ry * math.sin(h) + avg_rx * math.cos(h))

		self._total_distance_cm += avg_dist

		return avg_dist

	@staticmethod
	def _axis_for_angle(angle_deg: float) -> str:
		"""Classify a wheel steer angle into the nearest translation-calibration axis."""
		bucket = round((angle_deg % 360) / 45.0) % 8
		if bucket in (0, 4):
			return "forward_axis"
		elif bucket in (2, 6):
			return "strafe_axis"
		return "diagonal_axis"
