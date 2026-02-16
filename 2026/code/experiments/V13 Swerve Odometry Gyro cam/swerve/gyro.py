import wpilib
from wpimath.geometry import Rotation2d


class NavXGyro:
	"""Wrapper for NavX MXP IMU (Inertial Measurement Unit)."""

	def __init__(self, spi_port: wpilib.SPI.Port = wpilib.SPI.Port.kMXP):
		"""
		Initialize NavX MXP gyro.

		Args:
			spi_port: SPI port where NavX is connected (default MXP port)
		"""
		try:
			self.navx = wpilib.ADIS16448_IMU(wpilib.ADIS16448_IMU.IMUAxis.kZ, spi_port, wpilib.ADIS16448_IMU.CalibrationTime.k4s)
			self.is_connected = True
		except Exception as e:
			wpilib.reportError(f"Failed to initialize NavX: {e}", printTrace=True)
			self.is_connected = False

	def get_angle(self) -> Rotation2d:
		"""
		Get the current robot angle.

		Returns:
			Current robot rotation as Rotation2d
		"""
		if not self.is_connected:
			return Rotation2d()

		return Rotation2d.fromDegrees(self.navx.getAngle())

	def get_angle_degrees(self) -> float:
		"""
		Get the current robot angle in degrees.

		Returns:
			Current angle in degrees
		"""
		if not self.is_connected:
			return 0.0

		return self.navx.getAngle()

	def get_angular_velocity(self) -> float:
		"""
		Get the robot's angular velocity (rotation speed).

		Returns:
			Angular velocity in degrees per second
		"""
		if not self.is_connected:
			return 0.0

		return self.navx.getRate()

	def reset(self):
		"""Reset the gyro angle to 0 degrees."""
		if self.is_connected:
			self.navx.reset()

	def calibrate(self):
		"""Calibrate the gyro (should be called while robot is still)."""
		if self.is_connected:
			self.navx.calibrate()

	def is_ready(self) -> bool:
		"""
		Check if gyro is ready to use.

		Returns:
			True if gyro is connected and ready
		"""
		return self.is_connected
