from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("localization_fusion")
    default_ekf_map_yaml = os.path.join(pkg_share, "config", "ekf_map.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    ekf_map_yaml = LaunchConfiguration("ekf_map_yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "ekf_map_yaml",
                default_value=default_ekf_map_yaml,
                description="Path to robot_localization EKF config.",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_map",
                output="screen",
                parameters=[ekf_map_yaml, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
