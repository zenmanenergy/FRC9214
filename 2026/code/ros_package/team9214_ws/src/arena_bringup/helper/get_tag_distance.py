import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


def callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    distance = math.sqrt(x**2 + y**2)  # horizontal distance in map plane
    angle = math.degrees(math.atan2(y, x))  # bearing from map origin

    print(f"Distance: {distance:.2f}m, Angle: {angle:.1f}°, Height: {z:.2f}m")


def main():
    rclpy.init()
    node = rclpy.create_node("tag_distance_reader")
    node.create_subscription(PoseWithCovarianceStamped, "/tag_global_pose", callback, 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
