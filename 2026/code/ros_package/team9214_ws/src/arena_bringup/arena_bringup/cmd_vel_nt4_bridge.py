#!/usr/bin/env python3

from typing import Optional

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from ntcore import NetworkTableInstance
except ModuleNotFoundError as exc:
    NetworkTableInstance = None
    _NTCORE_IMPORT_ERROR = exc
else:
    _NTCORE_IMPORT_ERROR = None


class CmdVelNT4Bridge(Node):
    """Bridge cmd_vel-style velocity commands to NT4 topics."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_nt4_bridge")
        if NetworkTableInstance is None:
            raise RuntimeError(
                "Python module 'ntcore' is not installed. Install RobotPy NTCore "
                "(package 'pyntcore') in this environment."
            ) from _NTCORE_IMPORT_ERROR

        self.declare_parameter("cmd_vel_topic", "cmd_vel_nav")
        self.declare_parameter("mode_topic", "")
        self.declare_parameter("default_mode", "unknown")
        self.declare_parameter("cmd_source", "nav2")
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("localization_topic", "/tag_global_pose")
        self.declare_parameter("localization_timeout_sec", 1.0)
        self.declare_parameter("nav_status_topic", "/navigate_to_pose/_action/status")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("safety_stop_topic", "")
        self.declare_parameter("safety_state_topic", "")
        self.declare_parameter("heartbeat_hz", 10.0)
        self.declare_parameter("nt4_client_name", "arena_cmd_vel_bridge")
        self.declare_parameter("nt4_server", "10.92.14.2")
        self.declare_parameter("nt4_team", -1)
        self.declare_parameter("nt4_server_port", -1)
        self.declare_parameter("nt4_table", "ROS")
        self.declare_parameter("nt4_key_vx", "cmd_vel_x")
        self.declare_parameter("nt4_key_vy", "cmd_vel_y")
        self.declare_parameter("nt4_key_wz", "cmd_vel_theta")
        self.declare_parameter("nt4_key_array", "cmd_vel")
        self.declare_parameter("nt4_key_stamp", "cmd_vel_stamp")
        self.declare_parameter("nt4_key_mode", "status_mode")
        self.declare_parameter("nt4_key_cmd_source", "status_cmd_source")
        self.declare_parameter("nt4_key_cmd_valid", "status_cmd_valid")
        self.declare_parameter("nt4_key_cmd_age", "status_cmd_age_sec")
        self.declare_parameter("nt4_key_localization_ok", "status_localization_ok")
        self.declare_parameter("nt4_key_localization_age", "status_localization_age_sec")
        self.declare_parameter("nt4_key_nav_state", "status_nav_state")
        self.declare_parameter("nt4_key_nav_goal_active", "status_nav_goal_active")
        self.declare_parameter("nt4_key_nav_status_code", "status_nav_status_code")
        self.declare_parameter("nt4_key_safety_stop", "status_safety_stop")
        self.declare_parameter("nt4_key_safety_state", "status_safety_state")
        self.declare_parameter("nt4_key_odom_vx", "status_odom_vx")
        self.declare_parameter("nt4_key_odom_vy", "status_odom_vy")
        self.declare_parameter("nt4_key_odom_wz", "status_odom_wz")
        self.declare_parameter("nt4_key_heartbeat", "status_heartbeat")
        self.declare_parameter("nt4_key_status_stamp", "status_stamp")
        self.declare_parameter("publish_stamp", True)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        mode_topic = str(self.get_parameter("mode_topic").value)
        self._mode = str(self.get_parameter("default_mode").value)
        self._cmd_source = str(self.get_parameter("cmd_source").value)
        self._cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        localization_topic = str(self.get_parameter("localization_topic").value)
        self._localization_timeout_sec = float(self.get_parameter("localization_timeout_sec").value)
        nav_status_topic = str(self.get_parameter("nav_status_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        safety_state_topic = str(self.get_parameter("safety_state_topic").value)
        heartbeat_hz = float(self.get_parameter("heartbeat_hz").value)
        nt4_client_name = str(self.get_parameter("nt4_client_name").value)
        nt4_server = str(self.get_parameter("nt4_server").value)
        nt4_team = int(self.get_parameter("nt4_team").value)
        nt4_server_port = int(self.get_parameter("nt4_server_port").value)
        nt4_table = str(self.get_parameter("nt4_table").value)
        self._publish_stamp = bool(self.get_parameter("publish_stamp").value)

        key_vx = str(self.get_parameter("nt4_key_vx").value)
        key_vy = str(self.get_parameter("nt4_key_vy").value)
        key_wz = str(self.get_parameter("nt4_key_wz").value)
        key_array = str(self.get_parameter("nt4_key_array").value)
        key_stamp = str(self.get_parameter("nt4_key_stamp").value)
        key_mode = str(self.get_parameter("nt4_key_mode").value)
        key_cmd_source = str(self.get_parameter("nt4_key_cmd_source").value)
        key_cmd_valid = str(self.get_parameter("nt4_key_cmd_valid").value)
        key_cmd_age = str(self.get_parameter("nt4_key_cmd_age").value)
        key_localization_ok = str(self.get_parameter("nt4_key_localization_ok").value)
        key_localization_age = str(self.get_parameter("nt4_key_localization_age").value)
        key_nav_state = str(self.get_parameter("nt4_key_nav_state").value)
        key_nav_goal_active = str(self.get_parameter("nt4_key_nav_goal_active").value)
        key_nav_status_code = str(self.get_parameter("nt4_key_nav_status_code").value)
        key_safety_stop = str(self.get_parameter("nt4_key_safety_stop").value)
        key_safety_state = str(self.get_parameter("nt4_key_safety_state").value)
        key_odom_vx = str(self.get_parameter("nt4_key_odom_vx").value)
        key_odom_vy = str(self.get_parameter("nt4_key_odom_vy").value)
        key_odom_wz = str(self.get_parameter("nt4_key_odom_wz").value)
        key_heartbeat = str(self.get_parameter("nt4_key_heartbeat").value)
        key_status_stamp = str(self.get_parameter("nt4_key_status_stamp").value)

        self._last_cmd_time_sec: Optional[float] = None
        self._last_localization_time_sec: Optional[float] = None
        self._nav_status_code = GoalStatus.STATUS_UNKNOWN
        self._nav_state = "idle"
        self._nav_goal_active = False
        self._safety_stop = False
        self._safety_state = "unknown"
        self._odom_vx = 0.0
        self._odom_vy = 0.0
        self._odom_wz = 0.0
        self._heartbeat_counter = 0.0

        self._nt = NetworkTableInstance.getDefault()
        self._nt.startClient4(nt4_client_name)
        if nt4_team >= 0:
            self._nt.setServerTeam(nt4_team)
        elif nt4_server_port > 0:
            self._nt.setServer(nt4_server, nt4_server_port)
        else:
            self._nt.setServer(nt4_server)

        table = self._nt.getTable(nt4_table)
        self._pub_vx = table.getDoubleTopic(key_vx).publish()
        self._pub_vy = table.getDoubleTopic(key_vy).publish()
        self._pub_wz = table.getDoubleTopic(key_wz).publish()
        self._pub_array = table.getDoubleArrayTopic(key_array).publish()
        self._pub_mode = table.getStringTopic(key_mode).publish()
        self._pub_cmd_source = table.getStringTopic(key_cmd_source).publish()
        self._pub_cmd_valid = table.getBooleanTopic(key_cmd_valid).publish()
        self._pub_cmd_age = table.getDoubleTopic(key_cmd_age).publish()
        self._pub_localization_ok = table.getBooleanTopic(key_localization_ok).publish()
        self._pub_localization_age = table.getDoubleTopic(key_localization_age).publish()
        self._pub_nav_state = table.getStringTopic(key_nav_state).publish()
        self._pub_nav_goal_active = table.getBooleanTopic(key_nav_goal_active).publish()
        self._pub_nav_status_code = table.getIntegerTopic(key_nav_status_code).publish()
        self._pub_safety_stop = table.getBooleanTopic(key_safety_stop).publish()
        self._pub_safety_state = table.getStringTopic(key_safety_state).publish()
        self._pub_odom_vx = table.getDoubleTopic(key_odom_vx).publish()
        self._pub_odom_vy = table.getDoubleTopic(key_odom_vy).publish()
        self._pub_odom_wz = table.getDoubleTopic(key_odom_wz).publish()
        self._pub_heartbeat = table.getDoubleTopic(key_heartbeat).publish()
        self._pub_status_stamp = table.getDoubleTopic(key_status_stamp).publish()
        self._pub_stamp: Optional[object] = None
        if self._publish_stamp:
            self._pub_stamp = table.getDoubleTopic(key_stamp).publish()

        self._sub_cmd_vel = self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)
        self._sub_localization = self.create_subscription(
            PoseWithCovarianceStamped,
            localization_topic,
            self._localization_cb,
            10,
        )
        self._sub_nav_status = self.create_subscription(
            GoalStatusArray,
            nav_status_topic,
            self._nav_status_cb,
            10,
        )
        self._sub_odom = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        self._sub_mode = None
        if mode_topic:
            self._sub_mode = self.create_subscription(String, mode_topic, self._mode_cb, 10)

        self._sub_safety_stop = None
        if safety_stop_topic:
            self._sub_safety_stop = self.create_subscription(Bool, safety_stop_topic, self._safety_stop_cb, 10)

        self._sub_safety_state = None
        if safety_state_topic:
            self._sub_safety_state = self.create_subscription(
                String, safety_state_topic, self._safety_state_cb, 10
            )

        self._timer = self.create_timer(max(0.02, 1.0 / max(heartbeat_hz, 0.1)), self._publish_status)
        self.get_logger().info(
            f"Bridging '{cmd_vel_topic}' -> NT4 table '{nt4_table}' on server '{nt4_server}'"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _cmd_vel_cb(self, msg: Twist) -> None:
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)
        self._last_cmd_time_sec = self._now_sec()

        self._pub_vx.set(vx)
        self._pub_vy.set(vy)
        self._pub_wz.set(wz)
        self._pub_array.set([vx, vy, wz])

        if self._pub_stamp is not None:
            self._pub_stamp.set(self._last_cmd_time_sec)

    def _mode_cb(self, msg: String) -> None:
        self._mode = msg.data

    def _safety_stop_cb(self, msg: Bool) -> None:
        self._safety_stop = bool(msg.data)

    def _safety_state_cb(self, msg: String) -> None:
        self._safety_state = msg.data

    def _localization_cb(self, _msg: PoseWithCovarianceStamped) -> None:
        self._last_localization_time_sec = self._now_sec()

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_vx = float(msg.twist.twist.linear.x)
        self._odom_vy = float(msg.twist.twist.linear.y)
        self._odom_wz = float(msg.twist.twist.angular.z)

    def _nav_status_cb(self, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            self._nav_status_code = GoalStatus.STATUS_UNKNOWN
            self._nav_state = "idle"
            self._nav_goal_active = False
            return

        status = msg.status_list[-1].status
        self._nav_status_code = int(status)
        if status == GoalStatus.STATUS_ACCEPTED:
            self._nav_state = "accepted"
            self._nav_goal_active = True
        elif status == GoalStatus.STATUS_EXECUTING:
            self._nav_state = "executing"
            self._nav_goal_active = True
        elif status == GoalStatus.STATUS_CANCELING:
            self._nav_state = "canceling"
            self._nav_goal_active = True
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self._nav_state = "succeeded"
            self._nav_goal_active = False
        elif status == GoalStatus.STATUS_CANCELED:
            self._nav_state = "canceled"
            self._nav_goal_active = False
        elif status == GoalStatus.STATUS_ABORTED:
            self._nav_state = "aborted"
            self._nav_goal_active = False
        else:
            self._nav_state = "unknown"
            self._nav_goal_active = False

    def _publish_status(self) -> None:
        now = self._now_sec()
        cmd_age = now - self._last_cmd_time_sec if self._last_cmd_time_sec is not None else 1e9
        localization_age = (
            now - self._last_localization_time_sec if self._last_localization_time_sec is not None else 1e9
        )

        self._pub_mode.set(self._mode)
        self._pub_cmd_source.set(self._cmd_source)
        self._pub_cmd_valid.set(cmd_age <= self._cmd_timeout_sec)
        self._pub_cmd_age.set(cmd_age)
        self._pub_localization_ok.set(localization_age <= self._localization_timeout_sec)
        self._pub_localization_age.set(localization_age)
        self._pub_nav_state.set(self._nav_state)
        self._pub_nav_goal_active.set(self._nav_goal_active)
        self._pub_nav_status_code.set(self._nav_status_code)
        self._pub_safety_stop.set(self._safety_stop)
        self._pub_safety_state.set(self._safety_state)
        self._pub_odom_vx.set(self._odom_vx)
        self._pub_odom_vy.set(self._odom_vy)
        self._pub_odom_wz.set(self._odom_wz)
        self._pub_heartbeat.set(self._heartbeat_counter)
        self._pub_status_stamp.set(now)
        self._heartbeat_counter += 1.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelNT4Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
