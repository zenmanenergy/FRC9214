"""Localhost camera viewer for annotated AprilTag frames.

The viewer receives camera frames from NetworkTables, overlays AprilTag boxes,
and serves the result to a lightweight local web page.
"""

from __future__ import annotations

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

try:
    import cv2
except Exception:  # pragma: no cover - optional dependency
    cv2 = None

try:
    import numpy as np
except Exception:  # pragma: no cover - optional dependency
    np = None

from .network_tables_handler import NetworkTablesHandler
from .pose_estimator import Pose2D, blend_pose
from .vision_processor import VisionProcessor


class CameraViewer:
    def __init__(self, handler: Optional[NetworkTablesHandler] = None) -> None:
        self.handler = handler or NetworkTablesHandler(connect=True)
        self.vision = VisionProcessor()
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_pose: Optional[Pose2D] = None
        self.latest_source: str = "odometry"
        self.latest_weight: float = 0.0
        self._state: dict = {"apriltag_weight": 0.0}
        self._stop_event = threading.Event()
        self.template_path = Path(__file__).resolve().parents[2] / "dashboard" / "templates" / "vision_dashboard.html"

    def run(self, host: str = "127.0.0.1", port: int = 5000, poll_interval: float = 0.03) -> None:
        server = ThreadingHTTPServer((host, port), self._build_handler())
        server.app = self
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            while not self._stop_event.is_set():
                self._update_frame()
                time.sleep(poll_interval)
        finally:
            self._stop_event.set()
            server.shutdown()
            server.server_close()

    def _build_handler(self):
        app = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                if self.path == "/":
                    html = app.template_path.read_text(encoding="utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(html.encode("utf-8"))
                    return

                if self.path == "/video_feed":
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.end_headers()
                    while not app._stop_event.is_set():
                        frame = app.latest_frame
                        if frame is None:
                            time.sleep(0.02)
                            continue
                        success, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                        if success:
                            self.wfile.write(encoded.tobytes())
                        break
                    return

                self.send_response(404)
                self.end_headers()

            def log_message(self, format, *args):
                return

        return Handler

    def _update_frame(self) -> None:
        if cv2 is None or np is None:
            return
        frame = self.handler.read_camera_frame()
        if frame is None:
            return

        detections, annotated, best = self.vision.detect_frame(frame, draw_overlay=True)
        odom = self.handler.read_odometry_pose()
        if odom is None:
            odom = {"x": 0.0, "y": 0.0, "yaw": 0.0}

        tag_pose = best.pose if best is not None else None
        confidence = best.confidence if best is not None else 0.0
        blend = blend_pose(tag_pose, confidence, odom, dt_s=0.05, state=self._state)
        self.latest_pose = blend.pose
        self.latest_source = blend.source
        self.latest_weight = blend.weight

        cv2.putText(annotated, f"source={blend.source}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(annotated, f"weight={blend.weight:.2f}", (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if blend.pose is not None:
            cv2.putText(annotated, f"x={blend.pose.x:.2f} y={blend.pose.y:.2f}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        self.latest_frame = annotated
        self.handler.publish_blended_pose(
            {"x": blend.pose.x, "y": blend.pose.y, "yaw": blend.pose.yaw},
            blend.source,
            blend.confidence,
            blend.weight,
            time.time(),
        )


if __name__ == "__main__":
    viewer = CameraViewer()
    viewer.run()
