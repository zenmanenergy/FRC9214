#!/usr/bin/env python3

"""
Reads /SmartDashboard/odom_* and publishes /odom.
"""

import sys
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

import ntcore

# # Get python version to control imports
major_version = sys.version_info.major
minor_version = sys.version_info.minor

print("Major: ", major_version, "Minor: ", minor_version)

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    return q


class NtToRos(Node):
    def __init__(self):
        super().__init__("team9214")

        # self.declare_parameter("nt_server", "127.0.0.1")
        self.declare_parameter("nt_server", "10.92.14.2")
        self.declare_parameter("client_name", "nt_to_ros")
        self.declare_parameter("table", "SmartDashboard")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 50.0)

        nt_server = self.get_parameter("nt_server").value
        client_name = self.get_parameter("client_name").value
        table_name = self.get_parameter("table").value

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4(client_name)
        self.inst.setServer(nt_server)
        self.inst.startDSClient() # optional but recommended

        tbl = self.inst.getTable(table_name)

        self.sub_x = tbl.getDoubleTopic("odom_x_m").subscribe(0.0)
        self.sub_y = tbl.getDoubleTopic("odom_y_m").subscribe(0.0)
        self.sub_yaw = tbl.getDoubleTopic("odom_yaw_rad").subscribe(0.0)
        self.sub_vx = tbl.getDoubleTopic("odom_vx_mps").subscribe(0.0)
        self.sub_vy = tbl.getDoubleTopic("odom_vy_mps").subscribe(0.0)
        self.sub_wz = tbl.getDoubleTopic("odom_wz_radps").subscribe(0.0)

        self.pub = self.create_publisher(Odometry, "/odom", 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.publish_odom)

        self.get_logger().info(f"NT4→ROS: server={nt_server} table=/{table_name} -> /odom")

    def publish_odom(self):
        now = self.get_clock().now().to_msg()

        x = float(self.sub_x.get())
        y = float(self.sub_y.get())
        yaw = float(self.sub_yaw.get())
        vx = float(self.sub_vx.get())
        vy = float(self.sub_vy.get())
        wz = float(self.sub_wz.get())

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quat(yaw)

        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz

        # self.get_logger().info(f"NT4→ROS: position.x={msg.pose.pose.position.x} 
        #                         position.y={msg.pose.pose.position.y} 
        #                         position.x={msg.pose.pose.position.z}")
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = NtToRos()
    try:
        rclpy.spin(node)
    finally:
        node.inst.stopClient()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
