"""Unit tests for swerve.swerve_odometry.SwerveOdometry.

Uses FakeWheel test doubles instead of real hardware so the kinematics math
can be tested deterministically without needing simulated encoders.
"""

import math

import pytest

from swerve.swerve_odometry import SwerveOdometry
from swerve.unit_tests.fakes import FakeWheel
from swerve import swerve_config as config


def make_wheels():
	"""Four fake wheels positioned like the real WHEELS config."""
	return {
		name: FakeWheel(name, angle=180.0, drive_position=0.0)
		for name in config.WHEELS.keys()
	}


class TestPoseSettersGetters:

	def test_initial_pose_is_zero(self):
		odom = SwerveOdometry(make_wheels())
		assert odom.get_position() == (0.0, 0.0)
		assert odom.get_heading() == 0.0
		assert odom.get_total_distance() == 0.0

	def test_set_position(self):
		odom = SwerveOdometry(make_wheels())
		odom.set_position(12.0, -34.0)
		assert odom.get_position() == (12.0, -34.0)
		assert odom.get_x() == 12.0
		assert odom.get_y() == -34.0

	def test_set_heading_wraps_to_0_360(self):
		odom = SwerveOdometry(make_wheels())
		odom.set_heading(370.0)
		assert odom.get_heading() == pytest.approx(10.0)

		odom.set_heading(-10.0)
		assert odom.get_heading() == pytest.approx(350.0)

	def test_reset_zeroes_pose_and_distance(self):
		odom = SwerveOdometry(make_wheels())
		odom.set_position(5.0, 5.0)
		odom.set_heading(90.0)
		odom._total_distance_cm = 100.0

		odom.reset()

		assert odom.get_position() == (0.0, 0.0)
		assert odom.get_heading() == 0.0
		assert odom.get_total_distance() == 0.0

	def test_distance_helper_conversions(self):
		odom = SwerveOdometry(make_wheels())
		odom._total_distance_cm = 250.0
		assert odom.get_distance_traveled() == 250.0
		assert odom.get_distance_meters() == pytest.approx(2.5)


class TestUpdateKinematics:

	def test_no_movement_yields_zero_distance(self):
		wheels = make_wheels()
		odom = SwerveOdometry(wheels)
		avg_dist = odom.update()
		assert avg_dist == pytest.approx(0.0)
		assert odom.get_position() == pytest.approx((0.0, 0.0))

	def test_all_wheels_forward_moves_straight(self):
		# Hardware convention: 180 = forward for every wheel.
		wheels = make_wheels()
		odom = SwerveOdometry(wheels)

		rotations = 1.0
		for wheel in wheels.values():
			wheel.set_drive_position(rotations)

		avg_dist = odom.update()

		expected_dist_cm = (rotations / SwerveOdometry.DRIVE_GEAR_RATIO) * SwerveOdometry.WHEEL_CIRCUMFERENCE_CM
		assert avg_dist == pytest.approx(expected_dist_cm)

		x, y = odom.get_position()
		# Heading is 0 at start, so all forward motion should show up on one axis only.
		assert math.hypot(x, y) == pytest.approx(expected_dist_cm, rel=1e-3)

	def test_get_last_heading_delta_tracks_rotation(self):
		wheels = make_wheels()
		# Break symmetry: one wheel angled differently than the rest so the
		# kinematic cross-product sum used for heading is non-zero.
		wheels["front_right"].set_angle(90)

		odom = SwerveOdometry(wheels)
		for wheel in wheels.values():
			wheel.set_drive_position(1.0)

		odom.update()

		assert odom.get_last_heading_delta() != 0.0
