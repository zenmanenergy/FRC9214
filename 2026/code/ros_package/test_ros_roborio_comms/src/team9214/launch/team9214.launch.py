from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    venv_python = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", ".venv", "bin", "python"
    )
    # better: hardcode absolute path to your workspace venv
    venv_python = "/home/robots/source_code/first_robotics_comp/frc/ws/.venv/bin/python"

    return LaunchDescription([
        ExecuteProcess(
            cmd=[venv_python, "-m", "team9214.ros_to_nt"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[venv_python, "-m", "team9214.nt_to_ros"],
            output="screen",
        ),
    ])

# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package="team9214",
#             executable="ros_to_nt",
#             output="screen",
#             parameters=[{
#                 "nt_server": "10.92.14.2",
#                 "table": "SmartDashboard",
#                 "client_name": "ros_to_nt",
#             }],
#         ),
#         Node(
#             package="team9214",
#             executable="nt_to_ros",
#             output="screen",
#             parameters=[{
#                 "nt_server": "10.92.14.2",
#                 "table": "SmartDashboard",
#                 "client_name": "nt_to_ros",
#                 "publish_rate_hz": 50.0,
#                 "odom_frame": "odom",
#                 "base_frame": "base_link",
#             }],
#         ),
#     ])