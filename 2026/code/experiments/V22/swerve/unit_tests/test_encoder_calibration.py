"""Unit tests for swerve.encoder_calibration.EncoderCalibration.

Calibration is normally persisted to a fixed RoboRIO path (OFFSET_FILE).
Tests monkeypatch that module-level constant to a pytest tmp_path so no
real filesystem state is touched and each test starts from a clean file.
"""

import json

import pytest

import swerve.encoder_calibration as ec_module
from swerve.encoder_calibration import EncoderCalibration


@pytest.fixture
def cal_path(tmp_path, monkeypatch):
	path = tmp_path / "encoder_offsets.json"
	monkeypatch.setattr(ec_module, "OFFSET_FILE", str(path))
	return path


class TestLoadDefaults:

	def test_loads_defaults_when_file_missing(self, cal_path):
		cal = EncoderCalibration()
		assert cal.offsets == {
			"front_right": 0.0,
			"rear_right": 0.0,
			"rear_left": 0.0,
			"front_left": 0.0,
		}
		assert cal.get_pid_gains() == {"kp": 0.003, "ki": 0.005, "kd": 0.0001}
		assert cal.pid_tuning_history == []


class TestOffsets:

	def test_set_and_get_offset(self, cal_path):
		cal = EncoderCalibration()
		cal.set_offset("front_left", 45.0)
		assert cal.get_offset("front_left") == 45.0

	def test_get_offset_unknown_wheel_defaults_zero(self, cal_path):
		cal = EncoderCalibration()
		assert cal.get_offset("nonexistent") == 0.0

	def test_save_and_reload_roundtrip(self, cal_path):
		cal = EncoderCalibration()
		cal.set_offset("front_right", 12.5)
		cal.save_calibration()

		assert cal_path.exists()
		reloaded = EncoderCalibration()
		assert reloaded.get_offset("front_right") == pytest.approx(12.5)

	def test_save_offsets_is_alias_for_save_calibration(self, cal_path):
		cal = EncoderCalibration()
		cal.set_offset("rear_left", 7.0)
		cal.save_offsets()

		data = json.loads(cal_path.read_text())
		assert data["offsets"]["rear_left"] == pytest.approx(7.0)


class TestTuningHistoryAndRegression:

	def test_add_tuning_result_stores_history(self, cal_path):
		cal = EncoderCalibration()
		wheel_gains = {"front_left": {"kp": 0.01, "ki": 0.002, "kd": 0.0001}}
		cal.add_tuning_result(12.0, wheel_gains)

		assert len(cal.pid_tuning_history) == 1
		assert cal.pid_tuning_history[0]["wheel_gains"] == wheel_gains

	def test_interpolated_gains_uses_defaults_without_history(self, cal_path):
		cal = EncoderCalibration()
		gains = cal.get_interpolated_gains(12.0)
		assert gains == {"kp": 0.003, "ki": 0.005, "kd": 0.0001}

	def test_interpolated_gains_per_wheel_after_two_points(self, cal_path):
		cal = EncoderCalibration()
		wheel_names = ["front_left", "front_right", "rear_left", "rear_right"]

		def gains_for(v):
			return {name: {"kp": 0.001 * v, "ki": 0.0005 * v, "kd": 0.0001 * v} for name in wheel_names}

		cal.add_tuning_result(11.0, gains_for(11.0))
		cal.add_tuning_result(13.0, gains_for(13.0))

		result = cal.get_interpolated_gains(12.0)
		assert set(result.keys()) == set(wheel_names)
		for name in wheel_names:
			assert result[name]["kp"] == pytest.approx(0.012, abs=1e-6)

	def test_interpolated_gains_are_clamped(self, cal_path):
		cal = EncoderCalibration()
		wheel_names = ["front_left", "front_right", "rear_left", "rear_right"]
		huge_gains = {name: {"kp": 999.0, "ki": 999.0, "kd": 999.0} for name in wheel_names}
		cal.add_tuning_result(12.0, huge_gains)

		result = cal.get_interpolated_gains(12.0)
		for name in wheel_names:
			assert result[name]["kp"] <= 0.1
			assert result[name]["ki"] <= 0.5
			assert result[name]["kd"] <= 0.01

	def test_clear_tuning_history_resets_defaults(self, cal_path):
		cal = EncoderCalibration()
		cal.add_tuning_result(12.0, {"front_left": {"kp": 0.05, "ki": 0.01, "kd": 0.001}})

		cal.clear_tuning_history()

		assert cal.pid_tuning_history == []
		assert cal.pid_gains == {"kp": 0.003, "ki": 0.005, "kd": 0.0001}
		assert not hasattr(cal, "pid_regression") or cal.pid_regression == {}


class TestAlignmentAndNavigatorGains:

	def test_get_alignment_gains_defaults_empty(self, cal_path):
		cal = EncoderCalibration()
		assert cal.get_alignment_gains() == {}

	def test_set_and_get_navigator_rotation_gains(self, cal_path):
		cal = EncoderCalibration()
		cal.set_navigator_rotation_gains(0.01, 0.001, 0.0002)
		gains = cal.get_navigator_rotation_gains()
		assert gains == {"kp": 0.01, "ki": 0.001, "kd": 0.0002}


class TestLinearRegression:

	def test_linear_regression_fits_known_line(self, cal_path):
		# y = 2x + 1
		xs = [0.0, 1.0, 2.0, 3.0]
		ys = [1.0, 3.0, 5.0, 7.0]
		m, b = EncoderCalibration._linear_regression(xs, ys)
		assert m == pytest.approx(2.0)
		assert b == pytest.approx(1.0)

	def test_linear_regression_single_point_returns_flat_line(self, cal_path):
		m, b = EncoderCalibration._linear_regression([5.0], [42.0])
		assert m == 0
		assert b == 42.0

	def test_linear_regression_empty_returns_zero(self, cal_path):
		m, b = EncoderCalibration._linear_regression([], [])
		assert m == 0
		assert b == 0
