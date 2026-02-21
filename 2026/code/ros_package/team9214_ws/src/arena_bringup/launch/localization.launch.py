from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    ekf_map_yaml = LaunchConfiguration("ekf_map_yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true.",
        ),
        DeclareLaunchArgument(
            "ekf_map_yaml",
            default_value="",
            description="Path to robot_localization ekf_map YAML config.",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_map",
            output="screen",
            parameters=[ekf_map_yaml, {"use_sim_time": use_sim_time}],
        ),
    ])
