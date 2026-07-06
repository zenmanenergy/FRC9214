"""AprilTag utilities for the FRC 9214 vision stack.

The package now exposes a cleaner module layout:
- vision_processor.py: AprilTag detection only.
- pose_estimator.py: AprilTag + odometry blending.
- network_tables_handler.py: shared NetworkTables helpers.
- camera_viewer.py: localhost annotated frame viewer.
"""

from __future__ import annotations

import time
from typing import Any

try:
    from importlib_metadata import PackageNotFoundError, version
except Exception:  # pragma: no cover - Python 3.11+ compatibility
    from importlib.metadata import PackageNotFoundError, version

try:
    import ntcore
except Exception:  # pragma: no cover - optional on laptop/dev environments
    ntcore = None

try:
    from networktables import NetworkTablesInstance
except Exception:  # pragma: no cover - optional on laptop/dev environments
    NetworkTablesInstance = None

try:
    __version__ = version("frc-apriltags")
except PackageNotFoundError:
    __version__ = "0.0.0"

try:
    from .communications import NetworkCommunications
except Exception:  # pragma: no cover - keep import path usable without NT deps
    NetworkCommunications = Any  # type: ignore[assignment]

try:
    from .apriltags import Detector
except Exception:  # pragma: no cover - keep import path usable without WPILib deps
    Detector = Any  # type: ignore[assignment]

try:
    from .calibration import Calibrate
except Exception:  # pragma: no cover - keep import path usable without WPILib deps
    Calibrate = Any  # type: ignore[assignment]

try:
    from .camera import USBCamera
except Exception:  # pragma: no cover - keep import path usable without OpenCV deps
    USBCamera = Any  # type: ignore[assignment]

try:
    from .stream import Streaming
except Exception:  # pragma: no cover - keep import path usable without cscore deps
    Streaming = Any  # type: ignore[assignment]

from .network_tables_handler import NetworkTablesHandler
from .pose_estimator import Pose2D, PoseBlendResult, blend_pose
from .vision_processor import AprilTagDetection, VisionProcessor

try:
    from .camera_viewer import CameraViewer
except Exception:  # pragma: no cover - optional viewer dependency
    CameraViewer = None  # type: ignore[assignment]

__all__ = [
    "Detector",
    "Calibrate",
    "NetworkCommunications",
    "USBCamera",
    "Streaming",
    "NetworkTablesHandler",
    "Pose2D",
    "PoseBlendResult",
    "blend_pose",
    "AprilTagDetection",
    "VisionProcessor",
    "CameraViewer",
]


def startNetworkComms(teamNumber: int = 2199):
    """Start NetworkTables in the same shape used by the original package."""
    if ntcore is None or NetworkTablesInstance is None:
        return None

    teamStr = str(teamNumber).zfill(4)
    time.sleep(5)
    ntinst = NetworkTablesInstance.getDefault()
    ntinst.initialize(server="10." + teamStr[:2] + "." + teamStr[2:] + ".2")

    nt = ntcore.NetworkTableInstance.getDefault()
    nt.setServerTeam(teamNumber)
    nt.startClient4(__file__)
    ntinst.startClientTeam(teamNumber)
    return nt