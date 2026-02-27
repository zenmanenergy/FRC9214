import math
from typing import Dict, List, Optional, Tuple

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TagTestMonitor(Node):
    def __init__(self) -> None:
        super().__init__("tag_test_monitor")

        self.declare_parameter("detections_topic", "/camera_1/tag_detections")
        self.declare_parameter("camera_frame", "camera_1_optical_frame")
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("global_pose_topic", "/tag_global_pose")
        self.declare_parameter("lookup_timeout_sec", 0.05)
        self.declare_parameter("print_period_sec", 0.25)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.tag_family = str(self.get_parameter("tag_family").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.global_pose_topic = str(self.get_parameter("global_pose_topic").value)
        self.lookup_timeout_sec = float(self.get_parameter("lookup_timeout_sec").value)
        self.print_period_sec = float(self.get_parameter("print_period_sec").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_detection_frame: str = ""
        self.active_tag_ids: List[int] = []
        self.last_pose_msg: Optional[PoseWithCovarianceStamped] = None

        self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._detections_cb,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.global_pose_topic,
            self._global_pose_cb,
            10,
        )

        self.create_timer(self.print_period_sec, self._report)

        self.get_logger().info(
            "Monitoring tags on %s, camera_frame=%s, map_frame=%s, base_frame=%s"
            % (self.detections_topic, self.camera_frame, self.map_frame, self.base_frame)
        )

    def _detections_cb(self, msg: AprilTagDetectionArray) -> None:
        ids: List[int] = []
        for det in msg.detections:
            raw_id = det.id
            if isinstance(raw_id, (list, tuple)):
                if not raw_id:
                    continue
                ids.append(int(raw_id[0]))
            else:
                ids.append(int(raw_id))

        self.active_tag_ids = ids
        if msg.header.frame_id:
            self.last_detection_frame = msg.header.frame_id

    def _global_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.last_pose_msg = msg

    def _tag_frame_candidates(self, tag_id: int) -> List[str]:
        return [
            f"{self.tag_family}:{tag_id}",
            f"tag_{tag_id}",
            f"tag{tag_id}",
            str(tag_id),
        ]

    def _lookup_tag_from_camera(self, tag_id: int) -> Optional[Tuple[str, float, float, float, float, float]]:
        camera_frames = [self.camera_frame]
        if self.last_detection_frame and self.last_detection_frame not in camera_frames:
            camera_frames.insert(0, self.last_detection_frame)

        for cam_frame in camera_frames:
            for tag_frame in self._tag_frame_candidates(tag_id):
                try:
                    ts = self.tf_buffer.lookup_transform(
                        cam_frame,
                        tag_frame,
                        Time(),
                        timeout=Duration(seconds=self.lookup_timeout_sec),
                    )
                except Exception:
                    continue

                x = float(ts.transform.translation.x)
                y = float(ts.transform.translation.y)
                z = float(ts.transform.translation.z)
                range_3d = math.sqrt(x * x + y * y + z * z)
                range_xy = math.sqrt(x * x + y * y)
                bearing_deg = math.degrees(math.atan2(y, x))
                return tag_frame, range_3d, range_xy, bearing_deg, z, -y

        return None

    def _lookup_pose_from_tf(self) -> Optional[Tuple[float, float, float]]:
        try:
            ts = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.lookup_timeout_sec),
            )
        except Exception:
            return None

        x = float(ts.transform.translation.x)
        y = float(ts.transform.translation.y)
        yaw = _yaw_from_quaternion(
            float(ts.transform.rotation.x),
            float(ts.transform.rotation.y),
            float(ts.transform.rotation.z),
            float(ts.transform.rotation.w),
        )
        return x, y, yaw

    def _pose_from_global_pose_topic(self) -> Optional[Tuple[float, float, float, str]]:
        if self.last_pose_msg is None:
            return None
        p = self.last_pose_msg.pose.pose
        frame = self.last_pose_msg.header.frame_id or self.map_frame
        yaw = _yaw_from_quaternion(
            float(p.orientation.x),
            float(p.orientation.y),
            float(p.orientation.z),
            float(p.orientation.w),
        )
        return float(p.position.x), float(p.position.y), yaw, frame

    def _report(self) -> None:
        if self.active_tag_ids:
            for tag_id in self.active_tag_ids:
                result = self._lookup_tag_from_camera(tag_id)
                if result is None:
                    self.get_logger().info(
                        f"tag_id={tag_id}: detected but TF lookup failed (camera={self.camera_frame})"
                    )
                    continue

                tag_frame, range_3d, range_xy, bearing_deg, depth_m, height_up_m = result
                self.get_logger().info(
                    f"tag_id={tag_id} ({tag_frame}): range3d={range_3d:.2f}m, range_xy={range_xy:.2f}m, "
                    f"bearing={bearing_deg:.1f}deg, depth(z)={depth_m:.2f}m, height_up(-y)={height_up_m:.2f}m"
                )
        else:
            self.get_logger().info("No AprilTag detections in current frame.")

        tf_pose = self._lookup_pose_from_tf()
        if tf_pose is not None:
            x, y, yaw = tf_pose
            self.get_logger().info(
                f"arena_pose(TF {self.map_frame}->{self.base_frame}): x={x:.3f}m, y={y:.3f}m, yaw={yaw:.3f}rad"
            )
            return

        topic_pose = self._pose_from_global_pose_topic()
        if topic_pose is not None:
            x, y, yaw, frame = topic_pose
            self.get_logger().info(
                f"arena_pose(topic {self.global_pose_topic}, frame={frame}): x={x:.3f}m, y={y:.3f}m, yaw={yaw:.3f}rad"
            )
            return

        self.get_logger().info(
            f"arena_pose unavailable: no TF {self.map_frame}->{self.base_frame} and no {self.global_pose_topic} messages yet"
        )


def main() -> None:
    rclpy.init()
    node = TagTestMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
