#!/usr/bin/env python3
import math
import yaml
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray  # from apriltag_msgs

import tf2_ros


# ----------------------------
# Minimal rigid transform utils
# ----------------------------
def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n <= 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def quat_to_rot(qxyzw: np.ndarray) -> np.ndarray:
    # q = [x,y,z,w]
    x, y, z, w = quat_normalize(qxyzw)
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    return np.array([
        [1.0 - 2.0*(yy+zz), 2.0*(xy - wz),     2.0*(xz + wy)],
        [2.0*(xy + wz),     1.0 - 2.0*(xx+zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),     2.0*(yz + wx),     1.0 - 2.0*(xx+yy)],
    ], dtype=float)


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    # returns [x,y,z,w]
    tr = np.trace(R)
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
    return quat_normalize(np.array([x, y, z, w], dtype=float))


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return quat_normalize(np.array([x, y, z, w], dtype=float))


@dataclass
class SE3:
    R: np.ndarray  # 3x3
    t: np.ndarray  # 3,

    @staticmethod
    def identity() -> "SE3":
        return SE3(np.eye(3, dtype=float), np.zeros(3, dtype=float))

    def inv(self) -> "SE3":
        Rinv = self.R.T
        tinv = -Rinv @ self.t
        return SE3(Rinv, tinv)

    def __matmul__(self, other: "SE3") -> "SE3":
        return SE3(self.R @ other.R, self.R @ other.t + self.t)


def se3_from_transform_stamped(ts: TransformStamped) -> SE3:
    q = np.array([
        ts.transform.rotation.x,
        ts.transform.rotation.y,
        ts.transform.rotation.z,
        ts.transform.rotation.w
    ], dtype=float)
    R = quat_to_rot(q)
    t = np.array([
        ts.transform.translation.x,
        ts.transform.translation.y,
        ts.transform.translation.z
    ], dtype=float)
    return SE3(R, t)


def se3_from_pose(pose_msg) -> SE3:
    q = np.array([
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z,
        pose_msg.orientation.w
    ], dtype=float)
    R = quat_to_rot(q)
    t = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z], dtype=float)
    return SE3(R, t)


def yaw_from_R(R: np.ndarray) -> float:
    # Z-up yaw, ROS REP-103: yaw about +Z
    return math.atan2(R[1, 0], R[0, 0])


# ----------------------------
# Tag map loader
# ----------------------------
def load_tag_map(tag_map_yaml_path: str) -> Dict[int, SE3]:
    with open(tag_map_yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    tags = []
    if isinstance(data, dict):
        # Supported schemas:
        # 1) { tags: [...] }
        # 2) { apriltags: { tags: [...] } }
        tags = data.get("tags", [])
        if not tags:
            apriltags = data.get("apriltags", {})
            if isinstance(apriltags, dict):
                tags = apriltags.get("tags", [])

    if not isinstance(tags, list):
        tags = []

    out: Dict[int, SE3] = {}
    for entry in tags:
        tag_id = int(entry["id"])
        pose = entry["pose"]
        pos = pose["position"]
        rpy = pose.get("orientation_rpy", {"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
        q = rpy_to_quat(float(rpy["roll"]), float(rpy["pitch"]), float(rpy["yaw"]))
        R = quat_to_rot(q)
        t = np.array([float(pos["x"]), float(pos["y"]), float(pos["z"])], dtype=float)
        out[tag_id] = SE3(R, t)
    return out

# --------------------------------------------------------------
# Tag map loader for loading format in 
#   team9214_ws/src/arena_tag_map/config/example_arena_tags.yaml
# ---------------------------------------------------------------
# def load_tag_map(tag_map_yaml_path: str) -> Dict[int, SE3]:
#     with open(tag_map_yaml_path, "r", encoding="utf-8") as f:
#         data = yaml.safe_load(f)

#     tags = data["apriltags"]["tags"]
#     out: Dict[int, SE3] = {}
#     for entry in tags:
#         tag_id = int(entry["id"])
#         pose = entry["pose"]
#         pos = pose["position"]
#         rpy = pose.get("orientation_rpy", {"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
#         q = rpy_to_quat(float(rpy["roll"]), float(rpy["pitch"]), float(rpy["yaw"]))
#         R = quat_to_rot(q)
#         t = np.array([float(pos["x"]), float(pos["y"]), float(pos["z"])], dtype=float)
#         out[tag_id] = SE3(R, t)
#     return out

# ----------------------------
# Observer Node
# ----------------------------
class TagPoseObserver(Node):
    def __init__(self) -> None:
        super().__init__("tag_pose_observer")

        # Frames
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # Inputs / outputs
        self.declare_parameter("detection_topics", ["/camera_1/tag_detections"])
        self.declare_parameter("tag_map_yaml", "")
        self.declare_parameter("publish_topic", "/tag_global_pose")
        self.declare_parameter("camera_frame", "")

        # Gating / quality
        self.declare_parameter("min_decision_margin", 0.0)
        self.declare_parameter("max_pose_jump_m", 1.0)
        self.declare_parameter("max_yaw_jump_rad", 1.0)

        # Covariance model (simple)
        self.declare_parameter("position_std_m", 0.10)
        self.declare_parameter("yaw_std_rad", 0.15)

        # TF lookup
        self.declare_parameter("tf_timeout_sec", 0.05)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.detection_topics: List[str] = list(self.get_parameter("detection_topics").value)
        self.tag_map_yaml = self.get_parameter("tag_map_yaml").value
        self.publish_topic = self.get_parameter("publish_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value

        self.min_decision_margin = float(self.get_parameter("min_decision_margin").value)
        self.max_pose_jump_m = float(self.get_parameter("max_pose_jump_m").value)
        self.max_yaw_jump_rad = float(self.get_parameter("max_yaw_jump_rad").value)
        self.pos_std = float(self.get_parameter("position_std_m").value)
        self.yaw_std = float(self.get_parameter("yaw_std_rad").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)

        if not self.tag_map_yaml:
            raise RuntimeError("tag_map_yaml parameter must be set to a valid arena_tags.yaml file path")

        self.tag_map: Dict[int, SE3] = load_tag_map(self.tag_map_yaml)
        self.get_logger().info(f"Loaded {len(self.tag_map)} tags from: {self.tag_map_yaml}")

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.publish_topic, 10)

        # Subscriptions
        self.subs = []
        for topic in self.detection_topics:
            self.subs.append(
                self.create_subscription(
                    AprilTagDetectionArray,
                    topic,
                    self._on_detections,
                    10
                )
            )
            self.get_logger().info(f"Subscribing: {topic}")

        # Last accepted pose (for gating)
        self._last_T_map_base: Optional[SE3] = None

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        # Choose best candidate from this message, then gate against last accepted.
        best: Optional[Tuple[float, SE3, int, str]] = None
        # best = (quality, T_map_base, tag_id, camera_frame)

        for det in msg.detections:
            # apriltag_ros message variants may expose `id` as either:
            # - scalar int
            # - sequence (bundles / multi-id formats)
            raw_id = det.id
            if isinstance(raw_id, (list, tuple)):
                if len(raw_id) < 1:
                    continue
                tag_id = int(raw_id[0])
            else:
                tag_id = int(raw_id)
            if tag_id not in self.tag_map:
                continue

            decision_margin = float(getattr(det, "decision_margin", 0.0))
            if decision_margin < self.min_decision_margin:
                continue

            cam_frame = ""
            T_cam_tag: Optional[SE3] = None

            # Variant A: detection contains full pose (PoseWithCovarianceStamped)
            if hasattr(det, "pose"):
                cam_frame = det.pose.header.frame_id
                if cam_frame:
                    T_cam_tag = se3_from_pose(det.pose.pose.pose)

            # Variant B: detection is pixel-only. Derive T_cam_tag from TF published by apriltag node.
            if T_cam_tag is None:
                tag_frame_candidates = [
                    f"tag36h11:{tag_id}",
                    f"tag_{tag_id}",
                    f"tag{tag_id}",
                    str(tag_id),
                ]

                cam_frame_candidates = []
                if msg.header.frame_id:
                    cam_frame_candidates.append(msg.header.frame_id)
                if self.camera_frame:
                    cam_frame_candidates.append(self.camera_frame)
                if "camera_1_optical_frame" not in cam_frame_candidates:
                    cam_frame_candidates.append("camera_1_optical_frame")

                ts = None
                for cam_candidate in cam_frame_candidates:
                    for tag_frame in tag_frame_candidates:
                        try:
                            ts = self.tf_buffer.lookup_transform(
                                cam_candidate,
                                tag_frame,
                                rclpy.time.Time(),
                                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
                            )
                            cam_frame = cam_candidate
                            break
                        except Exception:
                            ts = None
                    if ts is not None:
                        break

                if ts is None:
                    self.get_logger().debug(
                        f"No camera->tag TF found for tag {tag_id}; "
                        f"camera frames tried={cam_frame_candidates}"
                    )
                    continue

                T_cam_tag = se3_from_transform_stamped(ts)

            # Known T_map_tag from tag map
            T_map_tag = self.tag_map[tag_id]

            # Need T_base_cam (base -> camera)
            try:
                ts = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    cam_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
                )
            except Exception as ex:
                self.get_logger().debug(f"TF lookup failed base->{cam_frame}: {ex}")
                continue

            T_base_cam = se3_from_transform_stamped(ts)

            # Compute:
            # T_map_base = T_map_tag * inv(T_cam_tag) * inv(T_base_cam)
            T_map_base = T_map_tag @ T_cam_tag.inv() @ T_base_cam.inv()

            quality = decision_margin
            if (best is None) or (quality > best[0]):
                best = (quality, T_map_base, tag_id, cam_frame)

        if best is None:
            return

        _, T_map_base, tag_id, cam_frame = best

        if not self._passes_gating(T_map_base):
            self.get_logger().debug(f"Rejected pose (gating) from tag {tag_id} cam {cam_frame}")
            return

        self._last_T_map_base = T_map_base
        self._publish_pose(T_map_base)

    def _passes_gating(self, T_map_base: SE3) -> bool:
        if self._last_T_map_base is None:
            return True

        dp = T_map_base.t - self._last_T_map_base.t
        dist = float(np.linalg.norm(dp[0:2]))  # 2D dist
        if dist > self.max_pose_jump_m:
            return False

        yaw_now = yaw_from_R(T_map_base.R)
        yaw_prev = yaw_from_R(self._last_T_map_base.R)
        dyaw = math.atan2(math.sin(yaw_now - yaw_prev), math.cos(yaw_now - yaw_prev))
        if abs(dyaw) > self.max_yaw_jump_rad:
            return False

        return True

    def _publish_pose(self, T_map_base: SE3) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.pose.position.x = float(T_map_base.t[0])
        msg.pose.pose.position.y = float(T_map_base.t[1])
        msg.pose.pose.position.z = float(T_map_base.t[2])

        q = rot_to_quat(T_map_base.R)
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])

        # Simple diagonal covariance: x,y,yaw
        cov = np.zeros((6, 6), dtype=float)
        cov[0, 0] = self.pos_std**2
        cov[1, 1] = self.pos_std**2
        cov[5, 5] = self.yaw_std**2
        msg.pose.covariance = cov.reshape(-1).tolist()

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TagPoseObserver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
