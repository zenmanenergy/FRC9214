import math
from typing import List, Optional, Sequence, Tuple

import cv2
from cv_bridge import CvBridge
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image


def _tag_id(det) -> str:
    raw_id = getattr(det, "id", None)
    if isinstance(raw_id, (list, tuple)):
        if not raw_id:
            return "?"
        return str(int(raw_id[0]))
    if raw_id is None:
        return "?"
    return str(int(raw_id))


def _extract_pose_position(det) -> Optional[Tuple[float, float, float]]:
    candidates = [
        ("pose", "pose", "pose", "position"),
        ("pose", "pose", "position"),
        ("pose", "position"),
    ]
    for path in candidates:
        cur = det
        ok = True
        for key in path:
            if not hasattr(cur, key):
                ok = False
                break
            cur = getattr(cur, key)
        if not ok:
            continue
        if all(hasattr(cur, k) for k in ("x", "y", "z")):
            return float(cur.x), float(cur.y), float(cur.z)
    return None


def _extract_corners(det) -> List[Tuple[int, int]]:
    corners = getattr(det, "corners", None)
    if corners is None:
        return []

    pts: List[Tuple[int, int]] = []
    for p in corners:
        if hasattr(p, "x") and hasattr(p, "y"):
            pts.append((int(round(float(p.x))), int(round(float(p.y)))))
        elif isinstance(p, Sequence) and len(p) >= 2:
            pts.append((int(round(float(p[0]))), int(round(float(p[1])))))

    return pts


def _extract_center(det, corners: List[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
    center = getattr(det, "center", None)
    if center is None:
        center = getattr(det, "centre", None)
    if center is not None and hasattr(center, "x") and hasattr(center, "y"):
        return int(round(float(center.x))), int(round(float(center.y)))

    if corners:
        x = int(round(sum(p[0] for p in corners) / len(corners)))
        y = int(round(sum(p[1] for p in corners) / len(corners)))
        return x, y

    return None


class AprilTagOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_overlay")

        self.declare_parameter("image_topic", "image_rect")
        self.declare_parameter("detections_topic", "tag_detections")
        self.declare_parameter("output_topic", "tag_overlay")
        self.declare_parameter("stale_detection_sec", 0.3)
        self.declare_parameter("line_thickness", 2)
        self.declare_parameter("font_scale", 0.5)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.stale_detection_sec = float(self.get_parameter("stale_detection_sec").value)
        self.line_thickness = int(self.get_parameter("line_thickness").value)
        self.font_scale = float(self.get_parameter("font_scale").value)

        self.bridge = CvBridge()
        self.latest_detections: Optional[AprilTagDetectionArray] = None
        self.latest_detection_time_ns: int = 0

        self.create_subscription(AprilTagDetectionArray, self.detections_topic, self._detections_cb, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)
        self.pub = self.create_publisher(Image, self.output_topic, 10)

        self.get_logger().info(
            f"Overlay enabled: image={self.image_topic}, detections={self.detections_topic}, output={self.output_topic}"
        )

    def _detections_cb(self, msg: AprilTagDetectionArray) -> None:
        self.latest_detections = msg
        self.latest_detection_time_ns = self.get_clock().now().nanoseconds

    def _image_cb(self, msg: Image) -> None:
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return

        now_ns = self.get_clock().now().nanoseconds
        has_fresh_detections = (
            self.latest_detections is not None
            and (now_ns - self.latest_detection_time_ns) / 1e9 <= self.stale_detection_sec
        )

        if has_fresh_detections:
            for det in self.latest_detections.detections:
                tag_id = _tag_id(det)
                corners = _extract_corners(det)
                center = _extract_center(det, corners)
                pos = _extract_pose_position(det)

                if len(corners) >= 4:
                    pts = corners[:4]
                    for i in range(4):
                        p0 = pts[i]
                        p1 = pts[(i + 1) % 4]
                        cv2.line(img, p0, p1, (0, 255, 0), self.line_thickness)

                if center is not None:
                    cv2.circle(img, center, 4, (0, 255, 255), -1)

                label = f"id={tag_id}"
                if pos is not None:
                    x, y, z = pos
                    dist = math.sqrt(x * x + y * y + z * z)
                    label += f" r={dist:.2f}m"

                if center is not None:
                    text_org = (center[0] + 8, center[1] - 8)
                elif corners:
                    text_org = (corners[0][0] + 8, corners[0][1] - 8)
                else:
                    text_org = (10, 24)

                cv2.putText(
                    img,
                    label,
                    text_org,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    self.font_scale,
                    (0, 255, 255),
                    max(1, self.line_thickness - 1),
                    cv2.LINE_AA,
                )

        out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = AprilTagOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
