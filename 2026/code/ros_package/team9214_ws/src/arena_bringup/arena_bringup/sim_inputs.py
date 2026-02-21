#!/usr/bin/env python3

import math
from typing import List, Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _parse_mode_schedule(raw: str) -> List[Tuple[str, float]]:
    out: List[Tuple[str, float]] = []
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    for part in parts:
        if ":" not in part:
            continue
        mode, sec = [x.strip() for x in part.split(":", 1)]
        try:
            duration = float(sec)
        except ValueError:
            continue
        if mode and duration > 0.0:
            out.append((mode, duration))
    return out


class SimInputs(Node):
    """Publishes simulated mode, tag pose, odom, and TF for pipeline testing."""

    def __init__(self) -> None:
        super().__init__("sim_inputs")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("publish_mode", True)
        self.declare_parameter("mode_topic", "robot_mode")
        self.declare_parameter("mode_schedule", "disabled:2,teleop:4,autonomous:10,disabled:2")
        self.declare_parameter("loop_mode_schedule", True)

        self.declare_parameter("publish_tag_pose", True)
        self.declare_parameter("tag_pose_topic", "/tag_global_pose")
        self.declare_parameter("tag_pose_frame_id", "map")
        self.declare_parameter("tag_pose_x", 0.0)
        self.declare_parameter("tag_pose_y", 0.0)
        self.declare_parameter("tag_pose_yaw", 0.0)

        self.declare_parameter("publish_odom", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("linear_velocity_x", 0.0)
        self.declare_parameter("angular_velocity_z", 0.0)
        self.declare_parameter("publish_tf", True)

        self._rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._publish_mode = bool(self.get_parameter("publish_mode").value)
        self._mode_topic = str(self.get_parameter("mode_topic").value)
        self._mode_schedule = _parse_mode_schedule(str(self.get_parameter("mode_schedule").value))
        self._loop_mode_schedule = bool(self.get_parameter("loop_mode_schedule").value)

        self._publish_tag_pose = bool(self.get_parameter("publish_tag_pose").value)
        self._tag_pose_topic = str(self.get_parameter("tag_pose_topic").value)
        self._tag_pose_frame_id = str(self.get_parameter("tag_pose_frame_id").value)
        self._tag_pose_x = float(self.get_parameter("tag_pose_x").value)
        self._tag_pose_y = float(self.get_parameter("tag_pose_y").value)
        self._tag_pose_yaw = float(self.get_parameter("tag_pose_yaw").value)

        self._publish_odom = bool(self.get_parameter("publish_odom").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._wheel_odom_topic = str(self.get_parameter("wheel_odom_topic").value)
        self._odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self._base_frame_id = str(self.get_parameter("base_frame_id").value)
        self._x = float(self.get_parameter("start_x").value)
        self._y = float(self.get_parameter("start_y").value)
        self._yaw = float(self.get_parameter("start_yaw").value)
        self._vx = float(self.get_parameter("linear_velocity_x").value)
        self._wz = float(self.get_parameter("angular_velocity_z").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        self._last_tick_sec = self.get_clock().now().nanoseconds / 1e9
        self._mode_index = 0
        self._mode_elapsed_sec = 0.0

        self._pub_mode = None
        if self._publish_mode:
            self._pub_mode = self.create_publisher(String, self._mode_topic, 10)

        self._pub_tag = None
        if self._publish_tag_pose:
            self._pub_tag = self.create_publisher(PoseWithCovarianceStamped, self._tag_pose_topic, 10)

        self._pub_odom = None
        self._pub_wheel_odom = None
        if self._publish_odom:
            self._pub_odom = self.create_publisher(Odometry, self._odom_topic, 10)
            if self._wheel_odom_topic and self._wheel_odom_topic != self._odom_topic:
                self._pub_wheel_odom = self.create_publisher(Odometry, self._wheel_odom_topic, 10)

        self._tfb = TransformBroadcaster(self) if self._publish_tf else None
        self._timer = self.create_timer(max(0.02, 1.0 / max(self._rate_hz, 0.1)), self._tick)

        self.get_logger().info("Sim inputs started")

    def _current_mode(self) -> str:
        if not self._mode_schedule:
            return "disabled"
        return self._mode_schedule[self._mode_index][0]

    def _advance_mode(self, dt: float) -> None:
        if not self._mode_schedule:
            return
        self._mode_elapsed_sec += dt
        _, dur = self._mode_schedule[self._mode_index]
        if self._mode_elapsed_sec < dur:
            return
        self._mode_elapsed_sec = 0.0
        if self._mode_index + 1 < len(self._mode_schedule):
            self._mode_index += 1
        elif self._loop_mode_schedule:
            self._mode_index = 0

    def _tick(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        dt = max(0.0, now_sec - self._last_tick_sec)
        self._last_tick_sec = now_sec

        # Integrate simple planar motion for odometry.
        self._x += self._vx * math.cos(self._yaw) * dt
        self._y += self._vx * math.sin(self._yaw) * dt
        self._yaw += self._wz * dt

        self._advance_mode(dt)

        if self._pub_mode is not None:
            mode_msg = String()
            mode_msg.data = self._current_mode()
            self._pub_mode.publish(mode_msg)

        if self._pub_tag is not None:
            tag_msg = PoseWithCovarianceStamped()
            tag_msg.header.stamp = self.get_clock().now().to_msg()
            tag_msg.header.frame_id = self._tag_pose_frame_id
            tag_msg.pose.pose.position.x = self._tag_pose_x
            tag_msg.pose.pose.position.y = self._tag_pose_y
            qx, qy, qz, qw = _quat_from_yaw(self._tag_pose_yaw)
            tag_msg.pose.pose.orientation.x = qx
            tag_msg.pose.pose.orientation.y = qy
            tag_msg.pose.pose.orientation.z = qz
            tag_msg.pose.pose.orientation.w = qw
            tag_msg.pose.covariance[0] = 0.02
            tag_msg.pose.covariance[7] = 0.02
            tag_msg.pose.covariance[35] = 0.05
            self._pub_tag.publish(tag_msg)

        odom_msg = None
        if self._pub_odom is not None:
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self._odom_frame_id
            odom_msg.child_frame_id = self._base_frame_id
            odom_msg.pose.pose.position.x = self._x
            odom_msg.pose.pose.position.y = self._y
            qx, qy, qz, qw = _quat_from_yaw(self._yaw)
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw
            odom_msg.twist.twist.linear.x = self._vx
            odom_msg.twist.twist.angular.z = self._wz
            self._pub_odom.publish(odom_msg)
            if self._pub_wheel_odom is not None:
                self._pub_wheel_odom.publish(odom_msg)

        if self._tfb is not None and odom_msg is not None:
            tf_odom_base = TransformStamped()
            tf_odom_base.header.stamp = odom_msg.header.stamp
            tf_odom_base.header.frame_id = self._odom_frame_id
            tf_odom_base.child_frame_id = self._base_frame_id
            tf_odom_base.transform.translation.x = self._x
            tf_odom_base.transform.translation.y = self._y
            tf_odom_base.transform.translation.z = 0.0
            tf_odom_base.transform.rotation = odom_msg.pose.pose.orientation
            self._tfb.sendTransform(tf_odom_base)

            tf_map_odom = TransformStamped()
            tf_map_odom.header.stamp = odom_msg.header.stamp
            tf_map_odom.header.frame_id = "map"
            tf_map_odom.child_frame_id = self._odom_frame_id
            tf_map_odom.transform.translation.x = 0.0
            tf_map_odom.transform.translation.y = 0.0
            tf_map_odom.transform.translation.z = 0.0
            tf_map_odom.transform.rotation.w = 1.0
            self._tfb.sendTransform(tf_map_odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimInputs()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

