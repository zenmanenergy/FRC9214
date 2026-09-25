"""Unit tests for swerve.swerve_drive.SwerveDrive.

SwerveDrive() is constructed once for the whole module since it allocates
real CAN IDs / DIO channels that cannot be re-allocated without closing them
(and wpilib.DutyCycleEncoder has no close()). An autouse fixture resets the
mutable logical state between tests so hardware is only ever created once.
"""

import json

import pytest

from swerve.swerve_drive import SwerveDrive


@pytest.fixture(scope="module")
def swerve():
	return SwerveDrive()


@pytest.fixture(autouse=True)
def reset_state(swerve):
	"""Reset mutable logical state before each test without recreating hardware."""
	swerve.stop_all()
	swerve.wheel_alignment_state.clear()
	swerve.movement_state = "idle"
	swerve.aligning = False
	swerve.following_path = False
	swerve.path_waypoints = None
	swerve.path_leg_index = 0
	swerve.recording_path = False
	swerve.recorded_positions = []
	swerve.odometry.reset()
	for pid in swerve.pid_controllers.values():
		pid.reset()
	yield


class TestMovementState:

	def test_initial_state_is_idle(self, swerve):
		assert swerve.get_movement_state() == "idle"
		assert swerve.is_moving() is False
		assert swerve.is_aligning() is False

	def test_drive_swerve_zero_input_stays_idle(self, swerve):
		swerve.drive_swerve(0.0, 0.0, 0.0)
		assert swerve.get_movement_state() == "idle"

	def test_drive_swerve_nonzero_input_sets_moving(self, swerve):
		swerve.drive_swerve(0.5, 0.0, 0.0)
		assert swerve.is_moving() is True

	def test_stop_all_returns_to_idle(self, swerve):
		swerve.drive_swerve(0.5, 0.0, 0.0)
		swerve.stop_all()
		assert swerve.get_movement_state() == "idle"
		assert swerve.wheel_alignment_state == {}


class TestPathFollowing:

	def test_is_path_complete_true_when_no_path(self, swerve):
		assert swerve.is_path_complete() is True

	def test_follow_path_starts_following(self, swerve):
		waypoints = [{"x": 0, "y": 0, "heading": 0}, {"x": 100, "y": 0, "heading": 0}]
		swerve.follow_path(waypoints, speed=0.5)

		assert swerve.is_path_complete() is False
		assert swerve.following_path is True
		assert swerve.path_waypoints == waypoints

	def test_stop_path_cancels_following(self, swerve):
		waypoints = [{"x": 0, "y": 0, "heading": 0}, {"x": 100, "y": 0, "heading": 0}]
		swerve.follow_path(waypoints, speed=0.5)

		swerve.stop_path()

		assert swerve.is_path_complete() is True
		assert swerve.path_waypoints is None

	def test_update_autonomous_without_path_stops_and_returns(self, swerve):
		swerve.update_autonomous()
		assert swerve.get_movement_state() == "idle"

	def test_update_autonomous_with_path_does_not_raise(self, swerve):
		waypoints = [{"x": 0, "y": 0, "heading": 0}, {"x": 100, "y": 50, "heading": 45}]
		swerve.follow_path(waypoints, speed=0.5)

		swerve.update_autonomous()  # Should not raise


class TestPathRecording:

	def test_recording_disabled_by_default(self, swerve):
		swerve.record_position()
		assert swerve.get_recorded_path() == []

	def test_start_recording_captures_positions(self, swerve):
		swerve.start_recording()
		swerve.record_position()

		recorded = swerve.get_recorded_path()
		assert len(recorded) == 1
		assert set(recorded[0].keys()) == {"x", "y", "heading", "timestamp", "distance"}

	def test_stop_recording_disables_capture(self, swerve):
		swerve.start_recording()
		swerve.record_position()
		swerve.stop_recording()
		swerve.record_position()

		assert len(swerve.get_recorded_path()) == 1

	def test_clear_recording_empties_positions(self, swerve):
		swerve.start_recording()
		swerve.record_position()
		swerve.clear_recording()

		assert swerve.get_recorded_path() == []
		assert swerve.recording_path is False

	def test_get_recorded_path_returns_a_copy(self, swerve):
		swerve.start_recording()
		swerve.record_position()

		recorded = swerve.get_recorded_path()
		recorded.append({"fake": "entry"})

		assert len(swerve.get_recorded_path()) == 1

	def test_export_recorded_path_false_when_empty(self, swerve, tmp_path):
		result = swerve.export_recorded_path(str(tmp_path / "empty.json"))
		assert result is False

	def test_export_recorded_path_writes_json(self, swerve, tmp_path):
		swerve.start_recording()
		swerve.record_position()
		out_file = tmp_path / "recorded.json"

		result = swerve.export_recorded_path(str(out_file))

		assert result is True
		assert out_file.exists()
		data = json.loads(out_file.read_text())
		assert data["waypoint_count"] == 1

	def test_publish_path_to_dashboard_does_not_raise(self, swerve):
		swerve.start_recording()
		swerve.record_position()
		swerve.follow_path([{"x": 0, "y": 0, "heading": 0}, {"x": 50, "y": 0, "heading": 0}], speed=0.3)

		swerve.publish_path_to_dashboard()  # Should not raise


class TestMotorCurrents:

	def test_update_motor_currents_does_not_raise(self, swerve):
		swerve.update_motor_currents()

	def test_get_motor_current_defaults_zero(self, swerve):
		swerve.update_motor_currents()
		assert swerve.get_motor_current("front_left") == pytest.approx(0.0)

	def test_get_motor_current_unknown_wheel_returns_zero(self, swerve):
		assert swerve.get_motor_current("does_not_exist") == 0.0

	def test_has_current_alert_defaults_false(self, swerve):
		assert swerve.has_current_alert() is False
		assert swerve.has_current_alert("front_left") is False


class TestDriveHelpers:

	def test_drive_straight_zero_speed_stops_wheels(self, swerve):
		swerve.drive_straight(0.0, target_angle=90.0)
		assert swerve.get_movement_state() == "idle"

	def test_drive_straight_nonzero_speed_does_not_raise(self, swerve):
		swerve.drive_straight(0.5, target_angle=0.0)
		assert swerve.get_movement_state() in ("moving", "aligning")

	def test_drive_to_heading_returns_bool(self, swerve):
		result = swerve.drive_to_heading(90.0)
		assert isinstance(result, bool)

	def test_drive_for_distance_completes_immediately_when_target_reached(self, swerve):
		swerve.odometry._total_distance_cm = 500.0
		done = swerve.drive_for_distance(0.5, target_distance_cm=100.0)
		assert done is True

	def test_drive_for_distance_not_done_when_far_from_target(self, swerve):
		swerve.odometry._total_distance_cm = 0.0
		done = swerve.drive_for_distance(0.5, target_distance_cm=200.0)
		assert done is False

	def test_drive_rotation_zero_input_sets_idle(self, swerve):
		swerve.drive_rotation(0.0)
		assert swerve.get_movement_state() == "idle"

	def test_drive_rotation_nonzero_input_does_not_raise(self, swerve):
		swerve.drive_rotation(0.5)

	def test_rotate_in_place_delegates_to_drive_rotation(self, swerve):
		swerve.rotate_in_place(0.3)  # Should not raise


class TestWheelAccessors:

	def test_get_wheel_angle_known_wheel(self, swerve):
		angle = swerve.get_wheel_angle("front_left")
		assert isinstance(angle, int)

	def test_get_wheel_angle_unknown_wheel_returns_negative_one(self, swerve):
		assert swerve.get_wheel_angle("nope") == -1

	def test_get_wheel_power_unknown_wheel_returns_zero(self, swerve):
		assert swerve.get_wheel_power("nope") == 0.0

	def test_set_wheel_drive_power_known_wheel(self, swerve):
		swerve.set_wheel_drive_power("front_left", 0.4)
		assert swerve.get_wheel_power("front_left") == pytest.approx(0.4)

	def test_set_wheel_drive_power_unknown_wheel_is_noop(self, swerve):
		swerve.set_wheel_drive_power("nope", 0.4)  # Should not raise

	def test_set_wheel_turn_power_unknown_wheel_is_noop(self, swerve):
		swerve.set_wheel_turn_power("nope", 0.4)  # Should not raise
