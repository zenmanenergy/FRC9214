"""NetworkTables helper for AprilTag and odometry data.

This module centralizes all NetworkTables reads and writes so the laptop-side
vision stack and the eventual RoboRIO port can share the same key layout.
"""

from __future__ import annotations

import os
import time
from typing import Any, Optional

try:
    import numpy as np
except Exception:  # pragma: no cover - numpy may be unavailable in some environments
    np = None

try:
    import cv2
except Exception:  # pragma: no cover - cv2 may be unavailable in some environments
    cv2 = None

try:
    from networktables import NetworkTables
except Exception:  # pragma: no cover - optional dependency
    NetworkTables = None


class NetworkTablesHandler:
    """Small compatibility wrapper around NetworkTables for laptop and RoboRIO."""

    def __init__(
        self,
        team_number: Optional[int] = None,
        server: Optional[str] = None,
        client_name: str = "frc_apriltags",
        table_prefix: str = "frc_apriltags",
        connect: bool = True,
    ) -> None:
        self.team_number = team_number or int(os.environ.get("FRC_TEAM", 9214))
        self.server = server
        self.client_name = client_name
        self.table_prefix = table_prefix
        self.table = None
        self.connected = False
        if connect:
            self.initialize()

    def initialize(self) -> None:
        if NetworkTables is None:
            return

        if self.server is not None:
            NetworkTables.initialize(server=self.server)
        else:
            NetworkTables.initialize()

        self.table = NetworkTables.getTable(self.table_prefix)
        self.connected = True

    def _entry(self, key: str):
        if self.table is None:
            return None
        return self.table.getEntry(key)

    # --- AprilTag publishing ---
    def publish_apriltag_detection(self, tag_id: int, pose: dict, confidence: float, timestamp: float) -> None:
        if self.table is None:
            return
        self._entry("vision/latest_tag_id").setDouble(float(tag_id))
        self._entry("vision/latest_tag_pose").setDoubleArray([pose.get("x", 0.0), pose.get("y", 0.0), pose.get("yaw", 0.0)])
        self._entry("vision/latest_tag_confidence").setDouble(float(confidence))
        self._entry("vision/latest_tag_timestamp").setDouble(float(timestamp))

    def publish_blended_pose(self, pose: dict, source: str, confidence: float, weight: float, timestamp: float) -> None:
        if self.table is None:
            return
        self._entry("pose/blended").setDoubleArray([pose.get("x", 0.0), pose.get("y", 0.0), pose.get("yaw", 0.0)])
        self._entry("pose/source").setString(source)
        self._entry("pose/confidence").setDouble(float(confidence))
        self._entry("pose/weight").setDouble(float(weight))
        self._entry("pose/timestamp").setDouble(float(timestamp))

    def publish_target_status(self, valid: bool, timestamp: float) -> None:
        if self.table is None:
            return
        self._entry("vision/target_valid").setBoolean(bool(valid))
        self._entry("vision/last_target_timestamp").setDouble(float(timestamp))

    # --- Camera frame transport ---
    def publish_camera_frame(self, frame: "np.ndarray", key: str = "vision/camera_frame", quality: int = 85) -> None:
        if self.table is None or cv2 is None or np is None or frame is None:
            return
        success, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not success:
            return
        self._entry(key).setRaw(encoded.tobytes())

    def read_camera_frame(self, key: str = "vision/camera_frame") -> Optional["np.ndarray"]:
        if self.table is None or cv2 is None or np is None:
            return None
        raw = self._entry(key).getRaw(None)
        if raw is None:
            return None
        try:
            payload = np.frombuffer(raw, dtype=np.uint8)
            frame = cv2.imdecode(payload, cv2.IMREAD_COLOR)
            return frame
        except Exception:
            return None

    # --- Odometry bridge ---
    def read_odometry_pose(self, key: str = "pose/odometry") -> Optional[dict]:
        if self.table is None:
            return None
        values = self._entry(key).getDoubleArray([])
        if not values:
            return None
        return {"x": float(values[0]), "y": float(values[1]), "yaw": float(values[2]) if len(values) > 2 else 0.0}

    def publish_odometry_pose(self, pose: dict, key: str = "pose/odometry") -> None:
        if self.table is None:
            return
        self._entry(key).setDoubleArray([pose.get("x", 0.0), pose.get("y", 0.0), pose.get("yaw", 0.0)])

    # --- Legacy compatibility helpers to match the old module API ---
    def setBestResult(self, result):
        tag_id = result[0]
        pose = result[1]
        self.publish_apriltag_detection(tag_id, pose, confidence=1.0, timestamp=time.time())

    def setTargetValid(self, valid: bool):
        self.publish_target_status(valid, time.time())

    def setDetectionTimeSec(self, time_sec: float):
        if self.table is not None:
            self._entry("vision/detection_time").setDouble(float(time_sec))

    def setBestResultId(self, tag_id: int):
        if self.table is not None:
            self._entry("vision/best_result_id").setDouble(float(tag_id))
