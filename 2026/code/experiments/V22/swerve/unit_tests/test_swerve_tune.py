"""Unit tests for swerve.swerve_tune.SwerveTuner.

Uses FakeWheel/FakeCalibration test doubles and drives the tuner's internal
state machine directly (rather than sleeping through real autotune timing)
so tests stay fast and deterministic.
"""

import pytest

from swerve.swerve_tune import SwerveTuner
from swerve.pid_controller import PIDController
from swerve.unit_tests.fakes import FakeWheel, FakeCalibration


def make_tuner():
	wheels = {
		"front_left": FakeWheel("front_left"),
		"front_right": FakeWheel("front_right"),
		"rear_left": FakeWheel("rear_left"),
		"rear_right": FakeWheel("rear_right"),
	}
	pid_controllers = {name: PIDController(0.01, 0.0, 0.0, name=name) for name in wheels}
	calibration = FakeCalibration()
	tuner = SwerveTuner(wheels, pid_controllers, calibration)
	return tuner, wheels, pid_controllers, calibration


class TestStartAndActive:

	def test_not_active_before_start(self):
		tuner, *_ = make_tuner()
		assert tuner.is_active() is False

	def test_start_activates_and_initializes_state(self):
		tuner, *_ = make_tuner()
		tuner.start()

		assert tuner.is_active() is True
		assert tuner.gains["current_index"] == 0
		assert tuner.gains["wheels"] == ["front_left", "front_right", "rear_left", "rear_right"]
		assert tuner.gains["results"] == []

	def test_update_is_noop_when_not_tuning(self):
		tuner, wheels, *_ = make_tuner()
		tuner.update()  # Should not raise even though gains is None
		assert wheels["front_left"].turn_power == 0.0


class TestAngleAndWheelProgression:

	def test_next_angle_or_finalize_advances_angle_index(self):
		tuner, *_ = make_tuner()
		tuner.start()

		tuner._next_angle_or_finalize()

		assert tuner.gains["current"]["angle_index"] == 1
		# State should reset for the new angle
		assert tuner.gains["current"]["kp"] == 0.009
		assert tuner.gains["current"]["sign_changes"] == 0

	def test_next_angle_or_finalize_finalizes_wheel_after_last_angle(self):
		tuner, wheels, pid_controllers, calibration = make_tuner()
		tuner.start()
		tuner.gains["current"]["angle_index"] = len(tuner.gains["current"]["angles"]) - 1

		tuner._next_angle_or_finalize()

		# Wheel finalized with fallback gains (no oscillation data), advances to next wheel
		assert tuner.gains["current_index"] == 1
		assert len(tuner.gains["results"]) == 1
		assert tuner.gains["results"][0]["wheel"] == "front_left"

	def test_finalize_wheel_with_no_oscillations_uses_fallback_gains(self):
		tuner, wheels, *_ = make_tuner()
		tuner.start()

		tuner._finalize_wheel()

		result = tuner.gains["results"][0]
		assert result["kp"] == pytest.approx(0.01)
		assert result["ki"] == pytest.approx(0.002)
		assert result["kd"] == pytest.approx(0.0001)
		assert wheels["front_left"].turn_power == 0.0

	def test_finalize_wheel_computes_gains_from_oscillation_data(self):
		tuner, *_ = make_tuner()
		tuner.start()
		tuner.gains["current"]["kc_list"] = [0.02, 0.02]
		tuner.gains["current"]["tc_list"] = [0.5, 0.5]

		tuner._finalize_wheel()

		result = tuner.gains["results"][0]
		assert result["kp"] == pytest.approx(0.6 * 0.02)
		assert result["ki"] == pytest.approx(0.8 * 0.02 / 0.5)
		assert result["kd"] == pytest.approx(0.075 * 0.02 * 0.5)

	def test_finalize_all_four_wheels_triggers_compute_final_gains(self):
		tuner, wheels, pid_controllers, calibration = make_tuner()
		tuner.start()

		for _ in range(4):
			tuner._finalize_wheel()

		# After the last wheel, tuning should be complete
		assert tuner.is_active() is False
		assert tuner.gains is None
		assert calibration.saved is True
		assert len(calibration.added_results) == 1


class TestComputeFinalGains:

	def test_compute_final_gains_applies_gains_to_pid_controllers(self):
		tuner, wheels, pid_controllers, calibration = make_tuner()
		tuner.start()
		tuner.gains["results"] = [
			{"wheel": "front_left", "kp": 0.05, "ki": 0.01, "kd": 0.001},
			{"wheel": "front_right", "kp": 0.06, "ki": 0.02, "kd": 0.002},
			{"wheel": "rear_left", "kp": 0.07, "ki": 0.03, "kd": 0.003},
			{"wheel": "rear_right", "kp": 0.08, "ki": 0.04, "kd": 0.004},
		]

		tuner._compute_final_gains()

		assert pid_controllers["front_left"].kp == pytest.approx(0.05)
		assert pid_controllers["rear_right"].kd == pytest.approx(0.004)
		assert calibration.saved is True
		voltage, wheel_gains = calibration.added_results[0]
		assert wheel_gains["front_right"]["ki"] == pytest.approx(0.02)


class TestPublishTuningHistory:

	def test_publish_tuning_history_does_not_raise_when_empty(self):
		tuner, *_ = make_tuner()
		tuner.publish_tuning_history()  # Should not raise

	def test_publish_tuning_history_does_not_raise_with_data(self):
		tuner, wheels, pid_controllers, calibration = make_tuner()
		calibration.pid_tuning_history.append({"battery_voltage": 12.0, "wheel_gains": {}})
		calibration.pid_regression = {"front_left": {"kp": {"m": 0, "b": 0.01}}}

		tuner.publish_tuning_history()  # Should not raise
