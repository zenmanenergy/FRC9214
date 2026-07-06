"""Pose blending between AprilTag detections and odometry.

The blended output is designed to be used by both the laptop visualization and
future RoboRIO-side code. It keeps AprilTag as the primary source when it is
visible and smoothly falls back to odometry when the tag disappears.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class PoseBlendResult:
    pose: Pose2D
    source: str
    weight: float
    confidence: float
    state: dict = field(default_factory=dict)


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _to_pose2d(pose: Any) -> Optional[Pose2D]:
    if pose is None:
        return None
    if isinstance(pose, Pose2D):
        return pose
    if isinstance(pose, dict):
        return Pose2D(float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), float(pose.get("yaw", 0.0)))
    if isinstance(pose, (tuple, list)) and len(pose) >= 3:
        return Pose2D(float(pose[0]), float(pose[1]), float(pose[2]))
    if hasattr(pose, "x") and hasattr(pose, "y") and hasattr(pose, "yaw"):
        return Pose2D(float(pose.x), float(pose.y), float(pose.yaw))
    if hasattr(pose, "X") and hasattr(pose, "Y") and hasattr(pose, "rotation"):
        rotation = pose.rotation()
        if hasattr(rotation, "radians"):
            yaw = rotation.radians()
        else:
            yaw = float(rotation.Z())
        return Pose2D(float(pose.X()), float(pose.Y()), yaw)
    return None


def blend_pose(
    apriltag_pose: Any,
    apriltag_confidence: float,
    odometry_pose: Any,
    dt_s: float,
    state: Optional[dict] = None,
    decay_seconds: float = 0.75,
    engage_seconds: float = 0.35,
    minimum_weight: float = 0.0,
) -> PoseBlendResult:
    """Blend an AprilTag pose with odometry using a smooth confidence-weighted transition."""
    odom = _to_pose2d(odometry_pose)
    if odom is None:
        odom = Pose2D(0.0, 0.0, 0.0)

    if state is None:
        state = {"apriltag_weight": 0.0}

    current_weight = float(state.get("apriltag_weight", 0.0))
    if apriltag_pose is not None and apriltag_confidence is not None and float(apriltag_confidence) > 0.2:
        target_weight = min(1.0, max(0.0, float(apriltag_confidence)))
        step = min(1.0, max(0.0, dt_s / max(float(engage_seconds), 1e-6)) * 1.5)
        current_weight = current_weight + (target_weight - current_weight) * step
    else:
        decay = max(0.0, dt_s / max(float(decay_seconds), 1e-6))
        current_weight = max(minimum_weight, current_weight - decay)

    weight = max(0.0, min(1.0, current_weight))

    apriltag = _to_pose2d(apriltag_pose)
    if apriltag is None:
        final_pose = odom
        source = "odometry"
        confidence = 0.0
    else:
        alpha = weight
        final_x = (1.0 - alpha) * odom.x + alpha * apriltag.x
        final_y = (1.0 - alpha) * odom.y + alpha * apriltag.y
        delta_yaw = _wrap_angle(apriltag.yaw - odom.yaw)
        final_yaw = odom.yaw + delta_yaw * alpha
        final_pose = Pose2D(final_x, final_y, final_yaw)
        if weight >= 0.7:
            source = "apriltag"
        elif weight <= 0.3:
            source = "odometry"
        else:
            source = "blended"
        confidence = float(apriltag_confidence) if apriltag_confidence is not None else 0.0

    state["apriltag_weight"] = weight
    state["last_source"] = source
    return PoseBlendResult(pose=final_pose, source=source, weight=weight, confidence=confidence, state=state)
