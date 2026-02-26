#!/usr/bin/env python3
"""
This assumes your bridge publishes:

    /cmd_vel (geometry_msgs/msg/Twist)
    /odom (nav_msgs/msg/Odometry)

    If your topic names differ, change the defaults or pass --cmd-topic / --odom-topic.

Run:

    ros2 run <your_pkg> shoreside_odom_plotter.py
    
    # or directly:
    python3 shoreside_odom_plotter.py --cmd-topic /cmd_vel --odom-topic /odom --csv /tmp/run.csv

"""
import argparse
import csv
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt


class OdomPlotter(Node):
    def __init__(self, args):
        super().__init__("shoreside_odom_plotter")

        self.cmd_topic = args.cmd_topic
        self.odom_topic = args.odom_topic
        self.maxlen = args.history

        self.log_csv_path = args.csv
        self.csv_file = None
        self.csv_writer = None
        if self.log_csv_path:
            self.csv_file = open(self.log_csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "t_wall_s",
                "cmd_vx", "cmd_vy", "cmd_wz",
                "odom_x", "odom_y", "odom_yaw",
                "odom_vx", "odom_vy", "odom_wz",
            ])

        self.t0 = time.time()

        # histories
        self.t = deque(maxlen=self.maxlen)
        self.cmd_vx = deque(maxlen=self.maxlen)
        self.cmd_vy = deque(maxlen=self.maxlen)
        self.cmd_wz = deque(maxlen=self.maxlen)

        self.ox = deque(maxlen=self.maxlen)
        self.oy = deque(maxlen=self.maxlen)
        self.oyaw = deque(maxlen=self.maxlen)

        self.ovx = deque(maxlen=self.maxlen)
        self.ovy = deque(maxlen=self.maxlen)
        self.owz = deque(maxlen=self.maxlen)

        # latest
        self._last_cmd = (0.0, 0.0, 0.0)

        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.get_logger().info(f"Subscribing cmd: {self.cmd_topic}, odom: {self.odom_topic}")

    def on_cmd(self, msg: Twist):
        self._last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)

    def on_odom(self, msg: Odometry):
        tw = time.time() - self.t0

        # cmd
        cvx, cvy, cwz = self._last_cmd

        # odom pose
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y

        # yaw from quaternion (x,y,z,w)
        q = msg.pose.pose.orientation
        # yaw = atan2(2(wz+xy), 1-2(y^2+z^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        oyaw = float(__import__("math").atan2(siny_cosp, cosy_cosp))

        # odom twist (body frame usually)
        ovx = msg.twist.twist.linear.x
        ovy = msg.twist.twist.linear.y
        owz = msg.twist.twist.angular.z

        # append
        self.t.append(tw)
        self.cmd_vx.append(cvx)
        self.cmd_vy.append(cvy)
        self.cmd_wz.append(cwz)

        self.ox.append(ox)
        self.oy.append(oy)
        self.oyaw.append(oyaw)

        self.ovx.append(ovx)
        self.ovy.append(ovy)
        self.owz.append(owz)

        # optional csv
        if self.csv_writer:
            self.csv_writer.writerow([tw, cvx, cvy, cwz, ox, oy, oyaw, ovx, ovy, owz])

    def close(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None


def run_plot(node: OdomPlotter, spin_hz: float):
    plt.ion()

    # time series figure
    fig_ts = plt.figure()
    ax1 = fig_ts.add_subplot(3, 1, 1)
    ax2 = fig_ts.add_subplot(3, 1, 2)
    ax3 = fig_ts.add_subplot(3, 1, 3)

    # XY figure
    fig_xy = plt.figure()
    ax_xy = fig_xy.add_subplot(1, 1, 1)

    last_warn_t = 0.0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0 / max(1.0, spin_hz))

            if len(node.t) < 2:
                plt.pause(0.01)
                continue

            # simple sanity warnings
            now = time.time()
            if now - last_warn_t > 2.0:
                last_warn_t = now
                if max(abs(v) for v in list(node.cmd_vx)[-10:]) < 1e-6 and max(abs(v) for v in list(node.cmd_wz)[-10:]) < 1e-6:
                    node.get_logger().warn("cmd_vel appears ~zero recently (could be OK if external mode idle).")
                if max(abs(v) for v in list(node.ovx)[-10:]) < 1e-6 and max(abs(v) for v in list(node.owz)[-10:]) < 1e-6:
                    node.get_logger().warn("odom twist appears ~zero recently (bridge/robot may not be publishing).")

            # ----- time series -----
            ax1.clear(); ax2.clear(); ax3.clear()

            ax1.plot(list(node.t), list(node.cmd_vx), label="cmd_vx")
            ax1.plot(list(node.t), list(node.cmd_vy), label="cmd_vy")
            ax1.plot(list(node.t), list(node.cmd_wz), label="cmd_wz")
            ax1.set_ylabel("cmd")
            ax1.legend(loc="upper right")

            ax2.plot(list(node.t), list(node.ovx), label="odom_vx")
            ax2.plot(list(node.t), list(node.ovy), label="odom_vy")
            ax2.plot(list(node.t), list(node.owz), label="odom_wz")
            ax2.set_ylabel("odom twist")
            ax2.legend(loc="upper right")

            ax3.plot(list(node.t), list(node.oyaw), label="yaw")
            ax3.set_ylabel("yaw (rad)")
            ax3.set_xlabel("t (s)")
            ax3.legend(loc="upper right")

            fig_ts.canvas.draw()
            fig_ts.canvas.flush_events()

            # ----- XY path -----
            ax_xy.clear()
            ax_xy.plot(list(node.ox), list(node.oy), label="odom path")
            ax_xy.set_xlabel("x (m)")
            ax_xy.set_ylabel("y (m)")
            ax_xy.set_aspect("equal", adjustable="datalim")
            ax_xy.legend(loc="upper right")

            fig_xy.canvas.draw()
            fig_xy.canvas.flush_events()

            plt.pause(0.01)

    finally:
        node.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cmd-topic", default="/cmd_vel")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--history", type=int, default=600)     # points
    p.add_argument("--spin-hz", type=float, default=50.0)
    p.add_argument("--csv", default="", help="optional csv path")
    args = p.parse_args()

    rclpy.init()
    node = OdomPlotter(args)
    try:
        run_plot(node, args.spin_hz)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
