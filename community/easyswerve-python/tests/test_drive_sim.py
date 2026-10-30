"""
Drivetrain behavior tests, run against physics.py with:

    python -m robotpy test

Each test drives the simulated robot with a simulated Xbox controller and
checks where odometry says it went. They catch direction, offset and
field-relative mistakes in software, before the robot is on the floor.
"""

import math

import pytest
from wpilib.simulation import XboxControllerSim

from easyswerve.drivetrain import configs

from easyswerve.sim.physics import findDrivetrain


def _drive(robot):
    return findDrivetrain(robot)


def test_forward_stick_drives_forward(control, robot):
    xbox = XboxControllerSim(0)
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        xbox.setLeftY(-1.0)  # stick pushed forward reads negative
        control.step_timing(seconds=1.0, autonomous=False, enabled=True)
        pose = _drive(robot).getPose()
        assert pose.X() > 1.0, f"expected forward motion, got {pose}"
        assert abs(pose.Y()) < 0.10, f"drifted sideways: {pose}"
        assert abs(pose.rotation().degrees()) < 3, f"rotated: {pose}"


def test_left_stick_left_strafes_left(control, robot):
    xbox = XboxControllerSim(0)
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        xbox.setLeftX(-1.0)  # stick pushed left reads negative
        control.step_timing(seconds=1.0, autonomous=False, enabled=True)
        pose = _drive(robot).getPose()
        assert pose.Y() > 1.0, f"expected motion to the left (+Y), got {pose}"
        assert abs(pose.X()) < 0.10, f"drifted forward/back: {pose}"


def test_right_stick_left_turns_counterclockwise(control, robot):
    xbox = XboxControllerSim(0)
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        xbox.setRightX(-0.5)
        control.step_timing(seconds=0.5, autonomous=False, enabled=True)
        drive = _drive(robot)
        assert drive.getHeading() > 20, f"expected CCW turn, heading {drive.getHeading()}"
        assert drive.getPose().translation().norm() < 0.15, "moved while spinning in place"


def test_field_relative_forward_after_turning(control, robot):
    """Turn the robot about 90 degrees, then push forward: field-relative
    driving should still go down the field (+X), not out the robot's nose."""
    xbox = XboxControllerSim(0)
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        xbox.setRightX(-0.5)
        drive = _drive(robot)
        for _ in range(100):
            control.step_timing(seconds=0.02, autonomous=False, enabled=True)
            if drive.getHeading() >= 85:
                break
        xbox.setRightX(0.0)
        control.step_timing(seconds=0.3, autonomous=False, enabled=True)
        start = drive.getPose()

        xbox.setLeftY(-1.0)
        control.step_timing(seconds=1.0, autonomous=False, enabled=True)
        end = drive.getPose()
        dx = end.X() - start.X()
        dy = end.Y() - start.Y()
        assert dx > 1.0, f"field-relative forward should be +X, moved dx={dx:.2f} dy={dy:.2f}"
        assert abs(dy) < 0.15, f"field-relative forward drifted, dy={dy:.2f}"


def test_x_lock_points_wheels_in_x(control, robot):
    xbox = XboxControllerSim(0)
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        xbox.setRightBumperButton(True)
        control.step_timing(seconds=0.5, autonomous=False, enabled=True)
        d = _drive(robot)
        expected = {"frontLeft": 45, "frontRight": -45, "rearLeft": -45, "rearRight": 45}
        for name, deg in expected.items():
            state = getattr(d, name).getState()
            diff = (state.angle.degrees() - deg + 90) % 180 - 90  # wheel at 45 == 225
            assert abs(diff) < 3, f"{name} at {state.angle.degrees():.1f}, expected {deg}"
            assert abs(state.speed) < 0.05, f"{name} still moving"


def test_odometry_tracks_a_square(control, robot):
    """Drive a square (legs about 1.2 m) with field-relative sticks and
    check odometry closes the loop. A bad module offset or kinematics value
    shows up here as a large closing error."""
    xbox = XboxControllerSim(0)
    legs = [(-0.5, 0.0), (0.0, -0.5), (0.5, 0.0), (0.0, 0.5)]  # fwd, left, back, right
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        for leftY, leftX in legs:
            xbox.setLeftY(leftY)
            xbox.setLeftX(leftX)
            control.step_timing(seconds=0.6, autonomous=False, enabled=True)
            xbox.setLeftY(0.0)
            xbox.setLeftX(0.0)
            control.step_timing(seconds=0.4, autonomous=False, enabled=True)
        pose = _drive(robot).getPose()
        assert pose.translation().norm() < 0.15, f"square did not close: {pose}"


@pytest.mark.parametrize("version", [1, 2])
def test_encoder_version_presets(version):
    assert configs.throughBoreEncoderPreset(version) is not None


def test_bad_encoder_version_rejected():
    with pytest.raises(ValueError):
        configs.throughBoreEncoderPreset(3)


def test_zero_steering_encoders(control, robot, monkeypatch, tmp_path):
    """Dashboard zeroing (roboRIO-wired encoders): refused while enabled,
    saved and applied while disabled."""
    from easyswerve.tuning import calibration

    monkeypatch.setenv(calibration.ENV_OVERRIDE, str(tmp_path / "cal.json"))
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        d = _drive(robot)
        if d.frontLeft.turningEncoder.location != "rio":
            pytest.skip("steering encoders are on the SPARKs")
        assert d.steeringEncodersReady()

        control.step_timing(seconds=0.2, autonomous=False, enabled=True)
        assert "disable" in d.zeroSteeringEncoders()
        assert not (tmp_path / "cal.json").exists()

        control.step_timing(seconds=0.2, autonomous=False, enabled=False)
        msg = d.zeroSteeringEncoders()
        assert msg.startswith("zeroed"), msg
        assert (tmp_path / "cal.json").exists()
        control.step_timing(seconds=0.6, autonomous=False, enabled=False)
        assert d.steeringEncodersReady()
