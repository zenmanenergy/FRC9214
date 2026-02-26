from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tag_map_yaml = LaunchConfiguration("tag_map_yaml")
    detection_topics = LaunchConfiguration("detection_topics")

    return LaunchDescription([
        DeclareLaunchArgument("tag_map_yaml", default_value=""),
        # For convenience, pass a Python-style list string if launching manually:
        # detection_topics:="['/camera_1/tag_detections','/camera_2/tag_detections']"
        DeclareLaunchArgument("detection_topics", default_value="['/camera_1/tag_detections']"),

        Node(
            package="tag_pose_observer",
            executable="tag_pose_observer",
            name="tag_pose_observer",
            output="screen",
            parameters=[
                {"tag_map_yaml": tag_map_yaml},
                {"detection_topics": detection_topics},
            ],
        )
    ])
