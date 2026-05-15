"""NavX2 MXP IMU wrapper for swerve drive.

The navX2 self-calibrates on power-up (~1-2 seconds). Keep the robot still
during that window. Yaw is automatically zeroed at end of calibration.

Heading convention matches swerve_odometry:
  0-360 degrees, counter-clockwise positive (standard math/field convention)

navX yaw is -180..180, CCW positive. We convert to 0..360.
If your robot's forward direction doesn't read 0 after power-up, call
zero_heading() once the robot is pointed at the field's zero direction.
"""
import wpilib
import navx
from wpilib import SmartDashboard


class SwerveIMU:

	def __init__(self, invert=False):
		"""
		Args:
			invert: Set True if your navX is mounted upside-down or yaw direction
			        is backwards relative to your robot's expected turning direction.
			        Test by rotating the robot CW and checking whether heading increases
			        or decreases. It should decrease for CW (since CCW = positive).
		"""
		self.ahrs = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
		self.invert = invert

	# ------------------------------------------------------------------
	# Status
	# ------------------------------------------------------------------

	def is_calibrating(self):
		return self.ahrs.isCalibrating()

	def is_connected(self):
		return self.ahrs.isConnected()

	def is_ready(self):
		return self.ahrs.isConnected() and not self.ahrs.isCalibrating()

	# ------------------------------------------------------------------
	# Heading
	# ------------------------------------------------------------------

	def get_heading(self):
		"""
		Returns heading in degrees, 0-360, CCW positive.
		Converts navX yaw (-180..180) to 0..360.
		"""
		yaw = self.ahrs.getYaw()
		if self.invert:
			yaw = -yaw
		return yaw % 360

	def zero_heading(self):
		"""Zero the yaw at the robot's current orientation."""
		self.ahrs.zeroYaw()

	# ------------------------------------------------------------------
	# Other axes (available for tilt detection, etc.)
	# ------------------------------------------------------------------

	def get_pitch(self):
		return self.ahrs.getPitch()

	def get_roll(self):
		return self.ahrs.getRoll()

	# ------------------------------------------------------------------
	# Integration with odometry
	# ------------------------------------------------------------------

	# Complementary filter weights.
	# IMU_WEIGHT controls how strongly the IMU absolute yaw pulls the fused
	# heading each loop.  1.0 = use IMU only.  0.0 = wheel kinematics only.
	# At 0.95 the fused heading tracks the IMU tightly but the wheel
	# kinematics heading still contributes, preventing the fused value from
	# jumping if the IMU glitches for a single loop.
	IMU_WEIGHT = 0.95

	def fuse_heading(self, odometry):
		"""
		Complementary filter: blend IMU absolute yaw with wheel-kinematics heading.
		Call this every loop AFTER odometry.update().

		When the IMU is healthy the fused heading sits very close to the IMU
		absolute yaw (IMU_WEIGHT = 0.95).  The remaining 5% from wheel
		kinematics keeps the value smooth and prevents single-loop glitches
		from causing a jump.

		When the IMU is not ready (still calibrating or disconnected) the
		function returns immediately, leaving the wheel-kinematics heading
		untouched so the robot still has a usable estimate.

		The blend uses the shortest angular path between the two estimates
		so there are no 0/360 wraparound artifacts.
		"""
		if not self.is_ready():
			# IMU unavailable - keep wheel-kinematics heading as-is
			return

		imu_heading = self.get_heading()
		wheel_heading = odometry.get_heading()

		# Shortest angular distance from wheel estimate to IMU reading
		diff = imu_heading - wheel_heading
		if diff > 180:
			diff -= 360
		elif diff < -180:
			diff += 360

		# Pull wheel heading toward IMU by IMU_WEIGHT fraction of the gap
		fused = (wheel_heading + self.IMU_WEIGHT * diff) % 360
		odometry.set_heading(fused)

	# ------------------------------------------------------------------
	# Dashboard
	# ------------------------------------------------------------------

	def publish_dashboard(self):
		SmartDashboard.putBoolean("IMU Connected", self.is_connected())
		SmartDashboard.putBoolean("IMU Calibrating", self.is_calibrating())
		SmartDashboard.putNumber("IMU Heading", round(self.get_heading(), 1))
		SmartDashboard.putNumber("IMU Pitch", round(self.get_pitch(), 1))
		SmartDashboard.putNumber("IMU Roll", round(self.get_roll(), 1))
