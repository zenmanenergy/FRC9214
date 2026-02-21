#!/usr/bin/env python3
"""
Subscribes /cmd_vel and publishes to /SmartDashboard/...
Adds cmd_vel_stamp_s for RoboRIO deadman logic.

On every /cmd_vel message: publish vx/vy/wz AND cmd_vel_stamp_s
If /cmd_vel goes silent for cmd_timeout_s: optionally publish zeros (configurable)
A timer publishes stamp periodically so the RoboRIO can distinguish “client alive but cmd quiet” vs “client dead”
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import ntcore


class RosToNt(Node):
    def __init__(self):
        super().__init__("team9214_ros_to_nt")

        # NetworkTables params
        self.declare_parameter("nt_server", "10.92.14.2")      # RoboRIO IP
        self.declare_parameter("client_name", "ros_to_nt")
        self.declare_parameter("table", "SmartDashboard")

        # Deadman / behavior params
        self.declare_parameter("cmd_timeout_s", 0.25)          # should match robot CMD_TIMEOUT_S
        self.declare_parameter("publish_zeros_on_timeout", True)
        self.declare_parameter("stamp_rate_hz", 20.0)          # periodic stamp updates (client heartbeat-ish)

        nt_server = self.get_parameter("nt_server").value
        client_name = self.get_parameter("client_name").value
        table_name = self.get_parameter("table").value

        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.publish_zeros_on_timeout = bool(self.get_parameter("publish_zeros_on_timeout").value)
        self.stamp_rate_hz = float(self.get_parameter("stamp_rate_hz").value)

        # NT init
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4(client_name)
        self.inst.setServer(nt_server)
        self.inst.startDSClient()  # optional but recommended

        tbl = self.inst.getTable(table_name)

        # Publishers for SmartDashboard keys
        self.pub_vx = tbl.getDoubleTopic("cmd_vel_vx_mps").publish()
        self.pub_vy = tbl.getDoubleTopic("cmd_vel_vy_mps").publish()
        self.pub_wz = tbl.getDoubleTopic("cmd_vel_wz_radps").publish()
        self.pub_stamp = tbl.getDoubleTopic("cmd_vel_stamp_s").publish()

        # Track last command time + last values
        self.last_cmd_wall = time.time()
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0

        # ROS subscription
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        # Periodic timers:
        #  1) publish stamps at a steady rate
        #  2) enforce timeout behavior
        self.create_timer(1.0 / max(1.0, self.stamp_rate_hz), self.on_stamp_timer)
        self.create_timer(0.05, self.on_timeout_timer)  # 20 Hz check

        self.get_logger().info(f"ROS→NT4: server={nt_server} table=/{table_name}")
        self.get_logger().info(
            f"cmd_timeout_s={self.cmd_timeout_s} publish_zeros_on_timeout={self.publish_zeros_on_timeout} stamp_rate_hz={self.stamp_rate_hz}"
        )

    def now_stamp(self) -> float:
        """
        We publish a 'wall-clock seconds' stamp.
        NOTE: RoboRIO uses FPGA timestamp for age computation. This works best if your
        robot code is in test mode (it overwrites stamp) OR you adjust robot logic to
        use a monotonic age that matches this clock domain.

        If you want perfect matching, we can change robot-side stale logic to use a
        simple "stamp increments" counter instead of comparing to FPGA time.
        """
        return time.time()

    def publish_cmd(self, vx: float, vy: float, wz: float):
        self.pub_vx.set(float(vx))
        self.pub_vy.set(float(vy))
        self.pub_wz.set(float(wz))
        self.pub_stamp.set(float(self.now_stamp()))

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd_wall = time.time()
        self.last_vx = float(msg.linear.x)
        self.last_vy = float(msg.linear.y)
        self.last_wz = float(msg.angular.z)

        self.publish_cmd(self.last_vx, self.last_vy, self.last_wz)

    def on_stamp_timer(self):
        """
        Periodic stamp updates. This helps indicate the client is alive.
        We also re-publish the last cmd values so the robot sees fresh stamps.
        """
        self.publish_cmd(self.last_vx, self.last_vy, self.last_wz)

    def on_timeout_timer(self):
        """
        If cmd topic goes quiet, optionally publish zeros.
        (This is separate from robot-side timeout.)
        """
        age = time.time() - self.last_cmd_wall
        if age > self.cmd_timeout_s and self.publish_zeros_on_timeout:
            # Only do it if we're not already zero to avoid spam
            if abs(self.last_vx) > 1e-9 or abs(self.last_vy) > 1e-9 or abs(self.last_wz) > 1e-9:
                self.last_vx, self.last_vy, self.last_wz = 0.0, 0.0, 0.0
                self.publish_cmd(0.0, 0.0, 0.0)

    def destroy(self):
        try:
            self.inst.stopClient()
        except Exception:
            pass


def main():
    rclpy.init()
    node = RosToNt()
    try:
        rclpy.spin(node)
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



# #!/usr/bin/env python3

# """
# Subscribes /cmd_vel and publishes to /SmartDashboard/....
# """

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# import ntcore

# class RosToNt(Node):
#     def __init__(self):
#         super().__init__("team9214")

#         # self.declare_parameter("nt_server", "127.0.0.1")
#         self.declare_parameter("nt_server", "10.92.14.2")
#         self.declare_parameter("client_name", "ros_to_nt")
#         self.declare_parameter("table", "SmartDashboard")

#         nt_server = self.get_parameter("nt_server").value
#         client_name = self.get_parameter("client_name").value
#         table_name = self.get_parameter("table").value

#         self.inst = ntcore.NetworkTableInstance.getDefault()
#         self.inst.startClient4(client_name)
#         self.inst.setServer(nt_server)
#         self.inst.startDSClient() # optional but recommended

#         tbl = self.inst.getTable(table_name)

#         # WPILib-ish keys under SmartDashboard
#         self.pub_vx = tbl.getDoubleTopic("cmd_vel_vx_mps").publish()
#         self.pub_vy = tbl.getDoubleTopic("cmd_vel_vy_mps").publish()
#         self.pub_wz = tbl.getDoubleTopic("cmd_vel_wz_radps").publish()

#         self.sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
#         self.get_logger().info(f"ROS→NT4: server={nt_server} table=/{table_name}")

#     def on_cmd_vel(self, msg: Twist):
#         self.pub_vx.set(float(msg.linear.x))
#         self.pub_vy.set(float(msg.linear.y))
#         self.pub_wz.set(float(msg.angular.z))

#         # self.get_logger().info(f"ROS→NT4: pub_vx={msg.linear.x} 
#         #                         pub_vy={msg.linear.y} 
#         #                         pub_vz={msg.linear.z}")


# def main():
#     rclpy.init()
#     node = RosToNt()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.inst.stopClient()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
