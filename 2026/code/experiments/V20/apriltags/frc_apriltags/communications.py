"""Backwards-compatible NetworkTables bridge for the AprilTag detector.

This wrapper keeps the existing detector interface while sending data through the
shared key layout implemented in network_tables_handler.py.
"""

from __future__ import annotations

import time

from .network_tables_handler import NetworkTablesHandler
from .Utilities import Logger


class NetworkCommunications:
    """Use this class to communicate with the RoboRIO over NetworkTables."""

    def __init__(self) -> None:
        self.logStatus = False
        self.handler = NetworkTablesHandler(connect=True)

        # Updates log
        Logger.logInfo("NetworkCommunications initialized", True)

    def setBestResultId(self, id: int):
        """Set the tag id of the best result."""
        self.handler._entry("vision/best_result_id").setDouble(float(id))

    def setBestResult(self, result):
        """Send the best result over NetworkTables using the shared key layout."""
        tag_id = int(result[0])
        pose = result[1]

        self.setBestResultId(tag_id)

        x, y, z = pose.X(), pose.Y(), pose.Z()
        roll, pitch, yaw = pose.rotation().X(), pose.rotation().Y(), pose.rotation().Z()
        data = (tag_id, x, y, z, roll, pitch, yaw)
        self.handler._entry("vision/latest_tag_pose").setDoubleArray([x, y, yaw])
        self.handler._entry("vision/latest_tag_id").setDouble(float(tag_id))
        self.handler._entry("vision/latest_tag_timestamp").setDouble(time.time())
        self.handler._entry("vision/latest_tag_confidence").setDouble(1.0)
        self.handler._entry("vision/last_detection_payload").setDoubleArray(list(data))

    def setTargetValid(self, tv: bool):
        """Set whether a valid target was detected."""
        self.handler.publish_target_status(tv, time.time())

    def setDetectionTimeSec(self, time_sec: float):
        """Set the time when a detection was made."""
        self.handler.setDetectionTimeSec(time_sec)

    def enableLogging(self):
        """Enable logging for this class."""
        self.logStatus = True