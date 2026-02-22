import rclpy
from geometry_msgs.msg import PoseStamped
import math

def callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    
    distance = math.sqrt(x**2 + y**2)  # horizontal distance
    angle = math.atan2(y, x) * 180 / math.pi  # angle in degrees
    
    print(f"Distance: {distance:.2f}m, Angle: {angle:.1f}°, Height: {z:.2f}m")

rclpy.init()
node = rclpy.create_node('tag_distance_reader')
sub = node.create_subscription(PoseStamped, '/tag_global_pose', callback, 10)
rclpy.spin(node)