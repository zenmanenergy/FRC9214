"""Unit tests for swerve.swerve_imu.SwerveIMU.

The real constructor talks to actual navX hardware/simulation, which has
non-deterministic startup calibration timing and spawns a background
calibration thread per instance. Tests replace the `.ahrs` attribute with a
FakeAHRS test double so behavior is deterministic, and reuse a small number
of module-scoped SwerveIMU instances to avoid piling up background threads.
"""

import pytest

from swerve.swerve_imu import SwerveIMU
from swerve.swerve_odometry import SwerveOdometry
from swerve.unit_tests.fakes import FakeAHRS, FakeWheel
from swerve import swerve_config as config


@pytest.fixture(scope="module")
def raw_imu():
	"""A single real SwerveIMU (invert=False), reused across tests."""
	return SwerveIMU(invert=False)


@pytest.fixture(scope="module")
def raw_inverted_imu():
	"""A single real SwerveIMU (invert=True), reused across tests."""
	return SwerveIMU(invert=True)


def make_imu(raw_imu, **fake_ahrs_kwargs):
	raw_imu.ahrs = FakeAHRS(**fake_ahrs_kwargs)
	return raw_imu


def make_odometry(heading: float = 0.0):
	wheels = {name: FakeWheel(name) for name in config.WHEELS.keys()}
	odom = SwerveOdometry(wheels)
	odom.set_heading(heading)
	return odom


class TestStatus:

	def test_is_calibrating_reflects_ahrs(self, raw_imu):
		imu = make_imu(raw_imu, calibrating=True)
		assert imu.is_calibrating() is True

	def test_is_connected_reflects_ahrs(self, raw_imu):
		imu = make_imu(raw_imu, connected=False)
		assert imu.is_connected() is False

	def test_is_ready_true_when_connected_and_not_calibrating(self, raw_imu):
		imu = make_imu(raw_imu, connected=True, calibrating=False)
		assert imu.is_ready() is True

	def test_is_ready_false_when_calibrating(self, raw_imu):
		imu = make_imu(raw_imu, connected=True, calibrating=True)
		assert imu.is_ready() is False

	def test_is_ready_false_when_disconnected(self, raw_imu):
		imu = make_imu(raw_imu, connected=False, calibrating=False)
		assert imu.is_ready() is False


class TestHeading:

	def test_get_heading_normalizes_negative_yaw(self, raw_imu):
		imu = make_imu(raw_imu, yaw=-90.0)
		assert imu.get_heading() == pytest.approx(270.0)

	def test_get_heading_passthrough_positive_yaw(self, raw_imu):
		imu = make_imu(raw_imu, yaw=45.0)
		assert imu.get_heading() == pytest.approx(45.0)

	def test_invert_flips_sign_before_normalizing(self, raw_inverted_imu):
		raw_inverted_imu.ahrs = FakeAHRS(yaw=45.0)
		assert raw_inverted_imu.get_heading() == pytest.approx(315.0)

	def test_zero_heading_resets_yaw(self, raw_imu):
		imu = make_imu(raw_imu, yaw=90.0)
		imu.zero_heading()
		assert imu.ahrs.zero_calls == 1
		assert imu.get_heading() == pytest.approx(0.0)


class TestPitchRoll:

	def test_get_pitch_and_roll_passthrough(self, raw_imu):
		imu = make_imu(raw_imu)
		assert imu.get_pitch() == pytest.approx(1.5)
		assert imu.get_roll() == pytest.approx(-2.5)


class TestFuseHeading:

	def test_fuse_heading_does_nothing_when_not_ready(self, raw_imu):
		imu = make_imu(raw_imu, connected=False)
		odom = make_odometry(heading=42.0)

		imu.fuse_heading(odom)

		assert odom.get_heading() == pytest.approx(42.0)

	def test_fuse_heading_pulls_wheel_heading_toward_imu(self, raw_imu):
		imu = make_imu(raw_imu, yaw=100.0, connected=True, calibrating=False)
		odom = make_odometry(heading=0.0)

		imu.fuse_heading(odom)

		# IMU_WEIGHT = 0.95, diff = 100, so fused = 0 + 0.95*100 = 95
		assert odom.get_heading() == pytest.approx(95.0)

	def test_fuse_heading_uses_shortest_path_across_wrap(self, raw_imu):
		imu = make_imu(raw_imu, yaw=350.0, connected=True, calibrating=False)
		odom = make_odometry(heading=10.0)

		imu.fuse_heading(odom)

		# Shortest path from 10 to 350 is -20 (not +340)
		expected = (10.0 + 0.95 * -20.0) % 360
		assert odom.get_heading() == pytest.approx(expected)

