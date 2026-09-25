"""Unit tests for swerve.heading_math (shortest_angle_diff, lerp_angle)."""

import pytest

from swerve.heading_math import shortest_angle_diff, lerp_angle


class TestShortestAngleDiff:

	def test_zero_diff(self):
		assert shortest_angle_diff(45.0, 45.0) == 0.0

	def test_simple_positive_diff(self):
		assert shortest_angle_diff(10.0, 20.0) == pytest.approx(10.0)

	def test_simple_negative_diff(self):
		assert shortest_angle_diff(20.0, 10.0) == pytest.approx(-10.0)

	def test_wraps_forward_across_zero(self):
		# 350 -> 10 is a 20 degree step forward through 360/0
		assert shortest_angle_diff(350.0, 10.0) == pytest.approx(20.0)

	def test_wraps_backward_across_zero(self):
		assert shortest_angle_diff(10.0, 350.0) == pytest.approx(-20.0)

	def test_exactly_180_stays_positive(self):
		# (b - a) % 360 == 180 is not > 180, so it is not flipped negative
		assert shortest_angle_diff(0.0, 180.0) == pytest.approx(180.0)

	def test_just_over_180_wraps_negative(self):
		assert shortest_angle_diff(0.0, 181.0) == pytest.approx(-179.0)

	def test_handles_inputs_outside_0_360(self):
		assert shortest_angle_diff(-10.0, 10.0) == pytest.approx(20.0)
		assert shortest_angle_diff(370.0, 10.0) == pytest.approx(0.0)


class TestLerpAngle:

	def test_t_zero_returns_start(self):
		assert lerp_angle(30.0, 90.0, 0.0) == pytest.approx(30.0)

	def test_t_one_returns_end(self):
		assert lerp_angle(30.0, 90.0, 1.0) == pytest.approx(90.0)

	def test_t_half_returns_midpoint(self):
		assert lerp_angle(0.0, 90.0, 0.5) == pytest.approx(45.0)

	def test_result_normalized_to_0_360(self):
		result = lerp_angle(0.0, -90.0, 1.0)
		assert 0.0 <= result < 360.0
		assert result == pytest.approx(270.0)

	def test_shortest_path_across_wraparound(self):
		# Shortest path from 350 to 10 goes forward through 0, not backward through 180
		result = lerp_angle(350.0, 10.0, 0.5)
		assert result == pytest.approx(0.0)
