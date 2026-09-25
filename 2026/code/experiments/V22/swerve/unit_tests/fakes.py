"""Lightweight test doubles used across the swerve unit test suite.

These avoid touching real CAN/DIO hardware resources so tests stay fast,
deterministic, and independent of the installed robotpy/rev/navx versions.
"""

from typing import Dict, List, Optional


class FakeEncoder:
	"""Stand-in for a SparkMax relative encoder."""

	def __init__(self, position: float = 0.0, velocity: float = 0.0) -> None:
		self.position = position
		self.velocity = velocity

	def getPosition(self) -> float:
		return self.position

	def getVelocity(self) -> float:
		return self.velocity


class FakeMotor:
	"""Stand-in for a rev.SparkMax motor controller."""

	def __init__(self) -> None:
		self.power = 0.0
		self.output_current = 0.0
		self.idle_mode = None
		self._encoder = FakeEncoder()

	def set(self, power: float) -> None:
		self.power = power

	def getOutputCurrent(self) -> float:
		return self.output_current

	def getEncoder(self) -> FakeEncoder:
		return self._encoder

	def setIdleMode(self, mode) -> None:
		self.idle_mode = mode


class FakeWheel:
	"""Stand-in for a SwerveWheel, used by odometry/tuner/drive tests."""

	def __init__(self, name: str, angle: float = 0.0, drive_position: float = 0.0) -> None:
		self.name = name
		self._angle = angle
		self._drive_position = drive_position
		self.turn_power = 0.0
		self.drive_power = 0.0
		self.stopped = False

	def get_angle(self) -> int:
		return int(round(self._angle)) % 360

	def get_raw_angle(self) -> float:
		return self._angle

	def set_angle(self, angle: float) -> None:
		self._angle = angle % 360

	def get_drive_position(self) -> float:
		return self._drive_position

	def set_drive_position(self, position: float) -> None:
		self._drive_position = position

	def set_turn_power(self, power: float) -> None:
		self.turn_power = power

	def set_drive_power(self, power: float) -> None:
		self.drive_power = power

	def get_drive_power(self) -> float:
		return self.drive_power

	def stop(self) -> None:
		self.stopped = True


class FakeAHRS:
	"""Stand-in for navx.AHRS, used by SwerveIMU tests."""

	def __init__(self, yaw: float = 0.0, calibrating: bool = False, connected: bool = True) -> None:
		self.yaw = yaw
		self.calibrating = calibrating
		self.connected = connected
		self.zero_calls = 0

	def getYaw(self) -> float:
		return self.yaw

	def isCalibrating(self) -> bool:
		return self.calibrating

	def isConnected(self) -> bool:
		return self.connected

	def zeroYaw(self) -> None:
		self.zero_calls += 1
		self.yaw = 0.0

	def getPitch(self) -> float:
		return 1.5

	def getRoll(self) -> float:
		return -2.5


class FakeCalibration:
	"""Stand-in for EncoderCalibration, used by SwerveTuner tests."""

	def __init__(self) -> None:
		self.pid_tuning_history: List[Dict] = []
		self.pid_regression: Dict = {}
		self.saved = False
		self.added_results = []

	def add_tuning_result(self, battery_voltage: float, wheel_gains: Dict) -> None:
		self.added_results.append((battery_voltage, wheel_gains))
		self.pid_tuning_history.append({
			"battery_voltage": battery_voltage,
			"wheel_gains": wheel_gains,
		})

	def save_calibration(self) -> None:
		self.saved = True
