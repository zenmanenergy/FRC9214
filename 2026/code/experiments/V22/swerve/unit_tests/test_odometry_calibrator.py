"""Unit tests for swerve.odometry_calibrator (RotationCalibrator, TranslationCalibrator).

These exercise the pure calibration math (multiplicative scale updates,
per-wheel drift-feedback bias, backlash accumulation) using real
SwerveOdometry/EncoderCalibration instances with fake hardware, per the
conventions in fakes.py / test_swerve_odometry.py / test_encoder_calibration.py.
"""

import pytest

import swerve.encoder_calibration as ec_module
from swerve.encoder_calibration import EncoderCalibration
from swerve.swerve_odometry import SwerveOdometry
from swerve.swerve_imu import SwerveIMU
from swerve.odometry_calibrator import RotationCalibrator, TranslationCalibrator, WHEEL_NAMES
from swerve.unit_tests.fakes import FakeAHRS, FakeWheel
from swerve import swerve_config as config


@pytest.fixture
def cal_path(tmp_path, monkeypatch):
	path = tmp_path / "encoder_offsets.json"
	monkeypatch.setattr(ec_module, "OFFSET_FILE", str(path))
	return path


@pytest.fixture(scope="module")
def raw_imu():
	return SwerveIMU(invert=False)


class FakeDrive:
	"""Minimal stand-in for SwerveDrive exposing only what the calibrators use."""

	def __init__(self, imu, odometry, calibration):
		self.imu = imu
		self.odometry = odometry
		self.calibration = calibration
		self.stop_calls = 0

	def stop_all(self):
		self.stop_calls += 1

	def rotate_in_place(self, power):
		pass

	def drive_straight(self, speed, angle):
		pass


def make_drive(cal_path, imu_fixture, heading=0.0):
	wheels = {name: FakeWheel(name, angle=180.0) for name in config.WHEELS.keys()}
	odometry = SwerveOdometry(wheels)
	odometry.set_heading(heading)
	calibration = EncoderCalibration()
	odometry.load_calibration(calibration)
	imu_fixture.ahrs = FakeAHRS(yaw=heading)
	imu_fixture.set_scale_factor(calibration.get_rotation_calibration()["imu_scale_factor"])
	return FakeDrive(imu_fixture, odometry, calibration)


class TestRotationCalibrator:

	def test_submit_residual_computes_scale_factors(self, cal_path, raw_imu):
		drive = make_drive(cal_path, raw_imu)
		cal = RotationCalibrator(drive, drive.calibration)

		cal.start_trial(n=2, speed_pct=50)
		# Simulate the robot under-reporting rotation: it thinks it went 700 deg
		# (both via IMU and wheel-odometry) when it should have gone 720 deg.
		cal._traveled_imu = 700.0
		cal._traveled_odom = 700.0

		entry = cal.submit_residual(residual_deg=0.0)

		assert entry["true_total"] == pytest.approx(720.0)
		assert entry["imu_correction_ratio"] == pytest.approx(720.0 / 700.0)
		# New scale factor should now be live-applied to the real IMU/odometry.
		assert drive.imu.scale_factor == pytest.approx(720.0 / 700.0)
		assert drive.odometry._rotation_scale_factor == pytest.approx(720.0 / 700.0)
		assert cal.state == "awaiting_reset"

	def test_scale_factor_compounds_correctly_across_trials(self, cal_path, raw_imu):
		"""A second trial, run after the first correction is already applied,
		must still converge toward the true absolute scale factor (see plan
		section 5 / multiplicative-update reasoning in odometry_calibrator.py).
		"""
		drive = make_drive(cal_path, raw_imu)
		cal = RotationCalibrator(drive, drive.calibration)

		# Trial 1: raw sensor under-reports by a factor of 720/700.
		cal.start_trial(n=2, speed_pct=50)
		cal._traveled_imu = 700.0
		cal._traveled_odom = 700.0
		cal.submit_residual(residual_deg=0.0)
		cal.confirm_reset()

		# Trial 2: same physical robot/sensor, but get_heading() now reports
		# through the already-applied scale factor. If the raw sensor is
		# perfectly linear, the reported total should already read ~true_total.
		cal.start_trial(n=2, speed_pct=50)
		cal._traveled_imu = 720.0
		cal._traveled_odom = 720.0
		entry2 = cal.submit_residual(residual_deg=0.0)

		# Correction ratio on trial 2 should be ~1.0 (already accurate),
		# and the absolute scale factor should remain ~720/700, not drift.
		assert entry2["imu_correction_ratio"] == pytest.approx(1.0)
		assert drive.imu.scale_factor == pytest.approx(720.0 / 700.0, rel=1e-6)


class TestTranslationCalibratorScale:

	def test_level1_forward_updates_average_scale(self, cal_path, raw_imu):
		drive = make_drive(cal_path, raw_imu)
		cal = TranslationCalibrator(drive, drive.calibration)

		cal.start_trial(level=1, x_meters=2.0, speed_pct=50)
		# Robot's odometry thought it drove 2.0m, tape measure shows only 1.8m.
		entry = cal.submit_result(measured_distance_m=1.8, perpendicular_drift_cm=0.0)

		expected_scale = 1.0 * (1.8 / 2.0)
		assert entry["new_avg_scale"] == pytest.approx(expected_scale)
		for name in WHEEL_NAMES:
			assert drive.odometry.get_wheel_scale(name, 0.5) == pytest.approx(expected_scale)

	def test_level1_drift_biases_left_right_wheels_oppositely(self, cal_path, raw_imu):
		drive = make_drive(cal_path, raw_imu)
		cal = TranslationCalibrator(drive, drive.calibration)

		cal.start_trial(level=1, x_meters=2.0, speed_pct=50)
		cal.submit_result(measured_distance_m=2.0, perpendicular_drift_cm=10.0)

		# Left wheels and right wheels should now differ (drift is not zero).
		left = drive.odometry.get_wheel_scale("front_left", 0.5)
		right = drive.odometry.get_wheel_scale("front_right", 0.5)
		assert left != pytest.approx(right)

	def test_level4_backlash_accumulates(self, cal_path, raw_imu):
		drive = make_drive(cal_path, raw_imu)
		cal = TranslationCalibrator(drive, drive.calibration)

		cal.start_trial(level=4, x_meters=1.0, speed_pct=50)
		entry = cal.submit_result(measured_distance_m=0.9, perpendicular_drift_cm=0.0)

		assert entry["shortfall_cm"] == pytest.approx(10.0)
		assert entry["backlash_cm"] == pytest.approx(10.0)
		assert drive.calibration.get_reversal_backlash()["forward_axis"] == pytest.approx(10.0)

	def test_level7_validation_records_error_without_changing_scale(self, cal_path, raw_imu):
		drive = make_drive(cal_path, raw_imu)
		cal = TranslationCalibrator(drive, drive.calibration)

		before = {name: drive.odometry.get_wheel_scale(name, 0.5) for name in WHEEL_NAMES}
		cal.start_trial(level=7, x_meters=1.0, speed_pct=50)
		pred_x, pred_y = cal.planned_path[-1]
		entry = cal.submit_result(measured_x_cm=pred_x + 3.0, measured_y_cm=pred_y)

		assert entry["error_cm"] == pytest.approx(3.0)
		for name in WHEEL_NAMES:
			assert drive.odometry.get_wheel_scale(name, 0.5) == pytest.approx(before[name])


class TestApplyDiscard:

	def test_discard_reverts_in_memory_changes(self, cal_path, raw_imu):
		drive = make_drive(cal_path, raw_imu)
		cal = RotationCalibrator(drive, drive.calibration)

		cal.start_trial(n=1, speed_pct=50)
		cal._traveled_imu = 350.0
		cal._traveled_odom = 350.0
		cal.submit_residual(residual_deg=0.0)
		assert drive.calibration.get_rotation_calibration()["imu_scale_factor"] != 1.0

		# Nothing was ever saved to disk, so discarding should restore defaults.
		drive.calibration.discard_odometry_calibration_changes()
		assert drive.calibration.get_rotation_calibration()["imu_scale_factor"] == 1.0
