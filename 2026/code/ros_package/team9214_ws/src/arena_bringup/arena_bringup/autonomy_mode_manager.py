#!/usr/bin/env python3

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class AutonomyModeManager(Node):
    """Starts/cancels Nav2 autonomous behavior based on robot mode topic."""

    def __init__(self) -> None:
        super().__init__("autonomy_mode_manager")

        self.declare_parameter("mode_topic", "robot_mode")
        self.declare_parameter("autonomous_mode_value", "autonomous")
        self.declare_parameter("cancel_on_mode_exit", True)
        self.declare_parameter("navigate_action_name", "navigate_to_pose")

        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("goal_x", 1.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("goal_behavior_tree", "")

        mode_topic = str(self.get_parameter("mode_topic").value)
        self._autonomous_mode_value = str(self.get_parameter("autonomous_mode_value").value).strip().lower()
        self._cancel_on_mode_exit = bool(self.get_parameter("cancel_on_mode_exit").value)
        action_name = str(self.get_parameter("navigate_action_name").value)

        self._goal_frame_id = str(self.get_parameter("goal_frame_id").value)
        self._goal_x = float(self.get_parameter("goal_x").value)
        self._goal_y = float(self.get_parameter("goal_y").value)
        self._goal_yaw = float(self.get_parameter("goal_yaw").value)
        self._goal_behavior_tree = str(self.get_parameter("goal_behavior_tree").value).strip()

        self._mode: Optional[str] = None
        self._goal_handle = None
        self._goal_active = False
        self._goal_pending = False

        self._action_client = ActionClient(self, NavigateToPose, action_name)
        
        # Subscribe to robot mode topic and attach callback
        self._sub_mode = self.create_subscription(String, mode_topic, self._mode_cb, 10)
        self.get_logger().info(
            f"Listening on '{mode_topic}', autonomous value '{self._autonomous_mode_value}', action '{action_name}'"
        )

    # Callback for robot mode_topic subscription
    def _mode_cb(self, msg: String) -> None:
        new_mode = msg.data.strip().lower()
        if not new_mode:
            return

        prev_mode = self._mode
        self._mode = new_mode
        is_auto = new_mode == self._autonomous_mode_value
        was_auto = prev_mode == self._autonomous_mode_value

        if is_auto and not was_auto:
            self.get_logger().info("Mode transitioned to autonomous, sending nav goal")
            self._send_goal()
        elif (not is_auto) and was_auto and self._cancel_on_mode_exit:
            self.get_logger().info("Mode exited autonomous, canceling active nav goal")
            self._cancel_goal()

    def _send_goal(self) -> None:
        if self._goal_pending or self._goal_active:
            self.get_logger().info("Goal already active/pending, not sending duplicate")
            return

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("NavigateToPose action server unavailable; cannot start autonomy goal")
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self._goal_frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self._goal_x
        pose.pose.position.y = self._goal_y
        qx, qy, qz, qw = _quat_from_yaw(self._goal_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        # Explicit behavior tree name to run for this goal, if supported by the server.
        if self._goal_behavior_tree:
            goal_msg.behavior_tree = self._goal_behavior_tree

        self._goal_pending = True
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        self._goal_pending = False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._goal_active = False
            self.get_logger().warn("Autonomy goal rejected by NavigateToPose server")
            return

        self._goal_handle = goal_handle
        self._goal_active = True
        self.get_logger().info("Autonomy goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        self._goal_active = False
        self._goal_handle = None
        result = future.result()
        if result is None:
            self.get_logger().warn("Autonomy goal finished without result")
            return

        status = int(result.status)
        self.get_logger().info(f"Autonomy goal finished with status={status}")

    def _cancel_goal(self) -> None:
        if self._goal_handle is None:
            return

        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_cb)

    def _cancel_done_cb(self, future) -> None:
        response = future.result()
        if response is None:
            self.get_logger().warn("Cancel request did not return a response")
            return
        self.get_logger().info(f"Cancel response count={len(response.goals_canceling)}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomyModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

