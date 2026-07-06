"""AprilTag detection pipeline.

This module performs image-based AprilTag detection and produces a lightweight
pose payload that can be consumed by the pose estimator and the camera viewer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

try:
    import cv2
except Exception:  # pragma: no cover - optional dependency
    cv2 = None

try:
    import numpy as np
except Exception:  # pragma: no cover - optional dependency
    np = None

try:
    import pupil_apriltags
except Exception:  # pragma: no cover - optional dependency
    pupil_apriltags = None


@dataclass
class AprilTagDetection:
    tag_id: int
    pose: dict
    confidence: float
    decision_margin: float
    center: Tuple[float, float]


class VisionProcessor:
    """Run AprilTag detection on a frame and return lightweight metadata."""

    def __init__(
        self,
        tag_size_meters: float = 0.1524,
        families: str = "tag16h5",
        max_hamming: int = 1,
        max_pose_error: float = 1e-3,
        min_decision_margin: float = 50.0,
        valid_tag_ids: Optional[Sequence[int]] = None,
    ) -> None:
        self.tag_size_meters = float(tag_size_meters)
        self.families = families
        self.max_hamming = max_hamming
        self.max_pose_error = max_pose_error
        self.min_decision_margin = float(min_decision_margin)
        self.valid_tag_ids = set(valid_tag_ids or range(1, 9))
        self.detector = None
        if pupil_apriltags is not None:
            self.detector = pupil_apriltags.Detector(families=self.families)

    def detect_frame(
        self,
        frame: np.ndarray,
        camera_matrix: Optional[np.ndarray] = None,
        draw_overlay: bool = False,
    ) -> Tuple[list[AprilTagDetection], np.ndarray, Optional[AprilTagDetection]]:
        if frame is None or np is None or cv2 is None:
            return [], frame, None

        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        if self.detector is None:
            return [], frame, None

        intrinsics = self._camera_intrinsics(camera_matrix)
        raw_detections = self.detector.detect(
            img=gray,
            estimate_tag_pose=True,
            camera_params=intrinsics,
            tag_size=self.tag_size_meters,
        )

        detections: list[AprilTagDetection] = []
        for raw in raw_detections:
            if int(raw.tag_id) not in self.valid_tag_ids:
                continue
            if raw.hamming > self.max_hamming:
                continue
            if raw.pose_err > self.max_pose_error:
                continue
            if raw.decision_margin < self.min_decision_margin:
                continue

            pose = self._pose_to_field_pose(raw.pose_R, raw.pose_t)
            confidence = self._confidence(raw)
            detection = AprilTagDetection(
                tag_id=int(raw.tag_id),
                pose=pose,
                confidence=confidence,
                decision_margin=float(raw.decision_margin),
                center=tuple(raw.center.astype(float)),
            )
            detections.append(detection)

            if draw_overlay and camera_matrix is not None:
                self._draw_overlay(frame, raw, camera_matrix)

        if detections:
            best = max(detections, key=lambda item: item.confidence)
        else:
            best = None

        return detections, frame, best

    def _camera_intrinsics(self, camera_matrix: Optional[np.ndarray]) -> Tuple[float, float, float, float]:
        if camera_matrix is None:
            return (640.0, 640.0, 320.0, 240.0)
        return (
            float(camera_matrix[0, 0]),
            float(camera_matrix[1, 1]),
            float(camera_matrix[0, 2]),
            float(camera_matrix[1, 2]),
        )

    def _pose_to_field_pose(self, rotation_matrix: np.ndarray, translation_vector: np.ndarray) -> dict:
        translation = np.asarray(translation_vector).reshape(-1)
        rotation = np.asarray(rotation_matrix)
        if translation.size < 3:
            return {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}

        x_cam = float(translation[2])
        y_cam = -float(translation[0])
        z_cam = -float(translation[1])
        yaw = float(math.atan2(rotation[1, 0], rotation[0, 0]))
        pitch = float(math.atan2(-rotation[2, 0], math.sqrt(rotation[2, 1] ** 2 + rotation[2, 2] ** 2)))
        roll = float(math.atan2(rotation[2, 1], rotation[2, 2]))
        return {"x": x_cam, "y": y_cam, "z": z_cam, "yaw": yaw, "pitch": pitch, "roll": roll}

    def _confidence(self, raw) -> float:
        confidence = max(0.0, float(raw.decision_margin) / 100.0)
        confidence = min(1.0, confidence)
        return confidence

    def _draw_overlay(self, frame: np.ndarray, raw, camera_matrix: np.ndarray) -> None:
        if camera_matrix is None or np is None or cv2 is None:
            return
        center = tuple(np.round(raw.center).astype(int).tolist())
        cv2.circle(frame, center, 6, (0, 255, 255), -1)
        cv2.putText(frame, f"tag {raw.tag_id}", (center[0] + 8, center[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
