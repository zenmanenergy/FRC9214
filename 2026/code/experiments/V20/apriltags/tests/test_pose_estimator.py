import math

from frc_apriltags.pose_estimator import Pose2D, blend_pose


def test_blend_moves_toward_apriltag_when_confident():
    state = {"apriltag_weight": 0.0}
    result = blend_pose(
        apriltag_pose=Pose2D(1.0, 2.0, 0.2),
        apriltag_confidence=0.9,
        odometry_pose=Pose2D(0.0, 0.0, 0.0),
        dt_s=0.2,
        state=state,
    )
    assert result.source == "apriltag"
    assert result.weight > 0.5
    assert result.pose.x > 0.0


def test_blend_falls_back_to_odometry_when_tag_is_lost():
    state = {"apriltag_weight": 0.9}
    result = blend_pose(
        apriltag_pose=None,
        apriltag_confidence=0.0,
        odometry_pose=Pose2D(1.0, 1.0, 0.1),
        dt_s=0.5,
        state=state,
        decay_seconds=0.5,
    )
    assert result.source == "odometry"
    assert result.weight < 0.2
    assert abs(result.pose.x - 1.0) < 1e-9
