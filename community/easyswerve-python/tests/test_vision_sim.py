"""
Vision in the simulator: photonlibpy's camera simulator looks at the simulated
field's AprilTags from the robot's true position (physics.py), and the robot's
position estimate should take those readings in.

Needs photonlibpy installed on the laptop, and the robot project wired as in
VISION_ENABLED = True in robot_settings.py. Run: python -m robotpy test
"""

import math

import pytest

pytest.importorskip("photonlibpy")

from easyswerve.sim.physics import findDrivetrain


def _poseFacingFirstTag(layout, distanceM=2.0):
    from wpimath.geometry import Pose2d, Rotation2d

    tag = layout.getTags()[0]
    p = tag.pose.toPose2d()
    yaw = p.rotation().radians()  # direction the tag faces, into the field
    x = p.X() + distanceM * math.cos(yaw)
    y = p.Y() + distanceM * math.sin(yaw)
    return Pose2d(x, y, Rotation2d(yaw + math.pi))  # robot faces the tag


def test_camera_estimates_are_fused_in_sim(control, robot):
    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        vision = getattr(findDrivetrain(robot), "vision", None)
        if vision is None or not vision.cameras:
            pytest.skip("vision is off (VISION_ENABLED = False in robot_settings.py)")

        drive = findDrivetrain(robot)
        drive.resetOdometry(_poseFacingFirstTag(vision.layout))
        before = drive.visionMeasurementCount

        control.step_timing(seconds=2.0, autonomous=False, enabled=True)

        cam = vision.cameras[0]
        seen = cam.accepted + cam.watched + sum(cam.rejected.values())
        assert seen > 0, "simulated camera saw no tags (check camera tilt and height in config.py)"
        assert drive.visionMeasurementCount > before, (
            f"camera saw tags but nothing was fused; rejected: {dict(cam.rejected)}, "
            f"mode: {cam.usableReason}"
        )


def test_robot_drives_with_no_camera(control, robot, monkeypatch):
    """With every camera disabled the robot must still drive normally."""
    from easyswerve.vision import config as C

    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        vision = getattr(findDrivetrain(robot), "vision", None)
        if vision is None:
            pytest.skip("drivetrain has no vision")
        vision.cameras = []  # as if photonlibpy were missing or the Pi absent
        from wpilib.simulation import XboxControllerSim

        xbox = XboxControllerSim(0)
        xbox.setLeftY(-1.0)
        control.step_timing(seconds=1.0, autonomous=False, enabled=True)
        assert findDrivetrain(robot).getPose().X() > 1.0
