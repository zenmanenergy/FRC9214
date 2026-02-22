#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

from action_msgs.msg import GoalStatus
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
        self.declare_parameter("goal_mode", "single_goal")
        self.declare_parameter("waypoints", "")
        self.declare_parameter("loop_on_success", True)
        self.declare_parameter("skip_failed_waypoint", True)

        mode_topic = str(self.get_parameter("mode_topic").value)
        self._autonomous_mode_value = str(self.get_parameter("autonomous_mode_value").value).strip().lower()
        self._cancel_on_mode_exit = bool(self.get_parameter("cancel_on_mode_exit").value)
        action_name = str(self.get_parameter("navigate_action_name").value)

        self._goal_frame_id = str(self.get_parameter("goal_frame_id").value)
        self._goal_x = float(self.get_parameter("goal_x").value)
        self._goal_y = float(self.get_parameter("goal_y").value)
        self._goal_yaw = float(self.get_parameter("goal_yaw").value)
        self._goal_behavior_tree = str(self.get_parameter("goal_behavior_tree").value).strip()
        raw_goal_mode = str(self.get_parameter("goal_mode").value).strip().lower()
        self._goal_mode = raw_goal_mode if raw_goal_mode in ("single_goal", "loop_waypoints") else "single_goal"
        self._waypoints_raw = str(self.get_parameter("waypoints").value).strip()
        self._loop_on_success = bool(self.get_parameter("loop_on_success").value)
        self._skip_failed_waypoint = bool(self.get_parameter("skip_failed_waypoint").value)

        self._mode: Optional[str] = None
        self._goal_handle = None
        self._goal_active = False
        self._goal_pending = False
        self._cancel_requested_for_mode_exit = False
        self._waypoints: List[Tuple[float, float, float]] = []
        self._waypoint_index = 0

        self._action_client = ActionClient(self, NavigateToPose, action_name)

        # Subscribe to robot mode topic and attach callback
        self._sub_mode = self.create_subscription(String, mode_topic, self._mode_cb, 10)
        self.get_logger().info(
            f"Listening on '{mode_topic}', autonomous value '{self._autonomous_mode_value}', "
            f"action '{action_name}', goal_mode '{self._goal_mode}'"
        )
        if raw_goal_mode not in ("single_goal", "loop_waypoints"):
            self.get_logger().warn(
                f"Unknown goal_mode '{raw_goal_mode}', defaulting to 'single_goal'"
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
            self._start_autonomous_behavior()
        elif (not is_auto) and was_auto and self._cancel_on_mode_exit:
            self.get_logger().info("Mode exited autonomous, canceling active nav goal")
            self._cancel_requested_for_mode_exit = True
            self._cancel_goal()

    def _start_autonomous_behavior(self) -> None:
        self._cancel_requested_for_mode_exit = False
        if self._goal_mode == "loop_waypoints":
            parse_error, parsed_waypoints = self._parse_waypoints(self._waypoints_raw)
            if parse_error:
                self.get_logger().error(parse_error)
                return
            if len(parsed_waypoints) < 2:
                self.get_logger().error(
                    "loop_waypoints mode requires at least 2 waypoints; "
                    f"got {len(parsed_waypoints)}"
                )
                return
            self._waypoints = parsed_waypoints
            self._waypoint_index = 0
            self._send_current_waypoint_goal()
            return
        self._send_goal(self._goal_x, self._goal_y, self._goal_yaw)

    def _send_current_waypoint_goal(self) -> None:
        if not self._waypoints:
            return
        x, y, yaw = self._waypoints[self._waypoint_index]
        self.get_logger().info(
            f"Loop waypoint {self._waypoint_index + 1}/{len(self._waypoints)}: "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )
        self._send_goal(x, y, yaw)

    def _send_goal(self, goal_x: float, goal_y: float, goal_yaw: float) -> None:
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
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        qx, qy, qz, qw = _quat_from_yaw(goal_yaw)
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
        self._maybe_continue_loop(status)

    def _maybe_continue_loop(self, status: int) -> None:
        if self._goal_mode != "loop_waypoints":
            return
        if self._mode != self._autonomous_mode_value:
            return
        if not self._waypoints:
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            if not self._loop_on_success:
                return
            self._waypoint_index = (self._waypoint_index + 1) % len(self._waypoints)
            self._send_current_waypoint_goal()
            return

        if status in (
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_UNKNOWN,
            GoalStatus.STATUS_CANCELED,
        ):
            if status == GoalStatus.STATUS_CANCELED and self._cancel_requested_for_mode_exit:
                self._cancel_requested_for_mode_exit = False
                return
            if self._skip_failed_waypoint:
                self._waypoint_index = (self._waypoint_index + 1) % len(self._waypoints)
                self.get_logger().warn(
                    "Waypoint goal did not succeed; skipping to next waypoint in loop"
                )
                self._send_current_waypoint_goal()
            return

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

    def _parse_waypoints(self, raw: str) -> Tuple[str, List[Tuple[float, float, float]]]:
        if not raw:
            return "loop_waypoints mode selected but 'waypoints' parameter is empty", []

        waypoints: List[Tuple[float, float, float]] = []
        entries = [entry.strip() for entry in raw.split(";") if entry.strip()]
        for i, entry in enumerate(entries):
            parts = [p.strip() for p in entry.split(",")]
            if len(parts) != 3:
                return (
                    f"Invalid waypoint at index {i}: '{entry}'. Expected format x,y,yaw",
                    [],
                )
            try:
                x = float(parts[0])
                y = float(parts[1])
                yaw = float(parts[2])
            except ValueError:
                return (
                    f"Invalid waypoint at index {i}: '{entry}'. Non-numeric value found",
                    [],
                )
            waypoints.append((x, y, yaw))
        return "", waypoints


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
