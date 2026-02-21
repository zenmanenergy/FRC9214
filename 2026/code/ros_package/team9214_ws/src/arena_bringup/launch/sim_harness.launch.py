import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("arena_bringup")
    default_nav_params = os.path.join(bringup_share, "params", "nav2_params.yaml")
    navigation_launch = os.path.join(bringup_share, "launch", "navigation.launch.py")

    launch_navigation = LaunchConfiguration("launch_navigation")
    nav_params_file = LaunchConfiguration("nav_params_file")
    nav_autostart = LaunchConfiguration("nav_autostart")

    mode_schedule = LaunchConfiguration("mode_schedule")
    sim_vx = LaunchConfiguration("sim_vx")
    sim_wz = LaunchConfiguration("sim_wz")
    tag_pose_x = LaunchConfiguration("tag_pose_x")
    tag_pose_y = LaunchConfiguration("tag_pose_y")
    tag_pose_yaw = LaunchConfiguration("tag_pose_yaw")

    enable_nt4_mode_bridge = LaunchConfiguration("enable_nt4_mode_bridge")
    nt4_mode_team = LaunchConfiguration("nt4_mode_team")
    nt4_mode_server = LaunchConfiguration("nt4_mode_server")
    nt4_mode_table = LaunchConfiguration("nt4_mode_table")
    nt4_mode_key = LaunchConfiguration("nt4_mode_key")

    enable_cmd_vel_nt4_bridge = LaunchConfiguration("enable_cmd_vel_nt4_bridge")
    nt4_team = LaunchConfiguration("nt4_team")
    nt4_server = LaunchConfiguration("nt4_server")
    nt4_table = LaunchConfiguration("nt4_table")

    enable_autonomy_mode_manager = LaunchConfiguration("enable_autonomy_mode_manager")
    autonomy_goal_x = LaunchConfiguration("autonomy_goal_x")
    autonomy_goal_y = LaunchConfiguration("autonomy_goal_y")
    autonomy_goal_yaw = LaunchConfiguration("autonomy_goal_yaw")
    autonomy_mode_value = LaunchConfiguration("autonomy_mode_value")

    return LaunchDescription([
        DeclareLaunchArgument("launch_navigation", default_value="true"),
        DeclareLaunchArgument("nav_params_file", default_value=default_nav_params),
        DeclareLaunchArgument("nav_autostart", default_value="true"),

        DeclareLaunchArgument("mode_schedule", default_value="disabled:2,teleop:4,autonomous:10,disabled:2"),
        DeclareLaunchArgument("sim_vx", default_value="0.0"),
        DeclareLaunchArgument("sim_wz", default_value="0.0"),
        DeclareLaunchArgument("tag_pose_x", default_value="0.0"),
        DeclareLaunchArgument("tag_pose_y", default_value="0.0"),
        DeclareLaunchArgument("tag_pose_yaw", default_value="0.0"),

        DeclareLaunchArgument("enable_nt4_mode_bridge", default_value="false"),
        DeclareLaunchArgument("nt4_mode_team", default_value="-1"),
        DeclareLaunchArgument("nt4_mode_server", default_value="10.92.14.2"),
        DeclareLaunchArgument("nt4_mode_table", default_value="ROS"),
        DeclareLaunchArgument("nt4_mode_key", default_value="robot_mode"),

        DeclareLaunchArgument("enable_cmd_vel_nt4_bridge", default_value="false"),
        DeclareLaunchArgument("nt4_team", default_value="-1"),
        DeclareLaunchArgument("nt4_server", default_value="10.92.14.2"),
        DeclareLaunchArgument("nt4_table", default_value="ROS"),

        DeclareLaunchArgument("enable_autonomy_mode_manager", default_value="true"),
        DeclareLaunchArgument("autonomy_mode_value", default_value="autonomous"),
        DeclareLaunchArgument("autonomy_goal_x", default_value="1.0"),
        DeclareLaunchArgument("autonomy_goal_y", default_value="0.0"),
        DeclareLaunchArgument("autonomy_goal_yaw", default_value="0.0"),

        Node(
            package="arena_bringup",
            executable="sim_inputs",
            name="sim_inputs",
            output="screen",
            parameters=[{
                "mode_schedule": mode_schedule,
                "linear_velocity_x": sim_vx,
                "angular_velocity_z": sim_wz,
                "tag_pose_x": tag_pose_x,
                "tag_pose_y": tag_pose_y,
                "tag_pose_yaw": tag_pose_yaw,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            condition=IfCondition(launch_navigation),
            launch_arguments={
                "params_file": nav_params_file,
                "autostart": nav_autostart,
                "enable_nt4_mode_bridge": enable_nt4_mode_bridge,
                "nt4_mode_team": nt4_mode_team,
                "nt4_mode_server": nt4_mode_server,
                "nt4_mode_table": nt4_mode_table,
                "nt4_mode_key": nt4_mode_key,
                "ros_mode_topic": "robot_mode",
                "enable_cmd_vel_nt4_bridge": enable_cmd_vel_nt4_bridge,
                "nt4_team": nt4_team,
                "nt4_server": nt4_server,
                "nt4_table": nt4_table,
                "cmd_vel_nt4_topic": "cmd_vel_nav",
                "enable_autonomy_mode_manager": enable_autonomy_mode_manager,
                "autonomy_mode_topic": "robot_mode",
                "autonomy_mode_value": autonomy_mode_value,
                "autonomy_goal_frame_id": "map",
                "autonomy_goal_x": autonomy_goal_x,
                "autonomy_goal_y": autonomy_goal_y,
                "autonomy_goal_yaw": autonomy_goal_yaw,
            }.items(),
        ),
    ])

