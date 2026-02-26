#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from ntcore import NetworkTableInstance
except ModuleNotFoundError as exc:
    NetworkTableInstance = None
    _NTCORE_IMPORT_ERROR = exc
else:
    _NTCORE_IMPORT_ERROR = None


class NT4ModeBridge(Node):
    """Bridge a NetworkTables mode string into a ROS2 String topic."""

    def __init__(self) -> None:
        super().__init__("nt4_mode_bridge")
        if NetworkTableInstance is None:
            raise RuntimeError(
                "Python module 'ntcore' is not installed. Install RobotPy NTCore "
                "(package 'pyntcore') in this environment."
            ) from _NTCORE_IMPORT_ERROR

        self.declare_parameter("nt4_client_name", "arena_mode_bridge")
        self.declare_parameter("nt4_server", "10.92.14.2")
        self.declare_parameter("nt4_team", -1)
        self.declare_parameter("nt4_server_port", -1)
        self.declare_parameter("nt4_table", "ROS")
        self.declare_parameter("nt4_mode_key", "robot_mode")
        self.declare_parameter("default_mode", "unknown")
        self.declare_parameter("ros_mode_topic", "robot_mode")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("publish_on_change_only", True)
        self.declare_parameter("normalize_lowercase", True)
        self.declare_parameter("strip_whitespace", True)

        nt4_client_name = str(self.get_parameter("nt4_client_name").value)
        nt4_server = str(self.get_parameter("nt4_server").value)
        nt4_team = int(self.get_parameter("nt4_team").value)
        nt4_server_port = int(self.get_parameter("nt4_server_port").value)
        nt4_table = str(self.get_parameter("nt4_table").value)
        nt4_mode_key = str(self.get_parameter("nt4_mode_key").value)
        self._default_mode = str(self.get_parameter("default_mode").value)
        ros_mode_topic = str(self.get_parameter("ros_mode_topic").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self._publish_on_change_only = bool(self.get_parameter("publish_on_change_only").value)
        self._normalize_lowercase = bool(self.get_parameter("normalize_lowercase").value)
        self._strip_whitespace = bool(self.get_parameter("strip_whitespace").value)

        self._pub_mode = self.create_publisher(String, ros_mode_topic, 10)
        self._last_mode = None

        self._nt = NetworkTableInstance.getDefault()
        self._nt.startClient4(nt4_client_name)
        if nt4_team >= 0:
            self._nt.setServerTeam(nt4_team)
        elif nt4_server_port > 0:
            self._nt.setServer(nt4_server, nt4_server_port)
        else:
            self._nt.setServer(nt4_server)

        table = self._nt.getTable(nt4_table)
        self._sub_mode = table.getStringTopic(nt4_mode_key).subscribe(self._default_mode)
        self._timer = self.create_timer(max(0.02, 1.0 / max(publish_hz, 0.1)), self._tick)

        self.get_logger().info(
            f"Bridging NT4 {nt4_table}/{nt4_mode_key} -> ROS topic '{ros_mode_topic}'"
        )

    def _normalize_mode(self, mode: str) -> str:
        out = mode
        if self._strip_whitespace:
            out = out.strip()
        if self._normalize_lowercase:
            out = out.lower()
        if not out:
            out = self._default_mode
        return out

    def _tick(self) -> None:
        mode = self._sub_mode.get()
        mode = self._normalize_mode(mode)

        if self._publish_on_change_only and mode == self._last_mode:
            return

        self._last_mode = mode
        msg = String()
        msg.data = mode
        self._pub_mode.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NT4ModeBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

