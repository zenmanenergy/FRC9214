
"""
Docstring for team9214_ws.src.arena_bringup.launch.bringup_multi_cam.launch

Summary:
    1. spins up an apriltag detector per camera namespace
    2. passes a real list of detection topics to the observer
    3. camera list is provided as a comma-separated string 
       (e.g., camera_1,camera_2,camera_3)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, GroupAction
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def _make_nodes(context, *args, **kwargs):
    arena_description_share = get_package_share_directory("arena_description")
    arena_tag_map_share = get_package_share_directory("arena_tag_map")
    tag_pose_observer_share = get_package_share_directory("tag_pose_observer")
    localization_share = get_package_share_directory("localization_fusion")
    bringup_share = get_package_share_directory("arena_bringup")

    urdf_xacro = LaunchConfiguration("urdf_xacro").perform(context)
    tag_map_yaml = LaunchConfiguration("tag_map_yaml").perform(context)
    observer_yaml = LaunchConfiguration("observer_yaml").perform(context)
    ekf_map_yaml = LaunchConfiguration("ekf_map_yaml").perform(context)
    ekf_odom_yaml = LaunchConfiguration("ekf_odom_yaml").perform(context)

    camera_names = LaunchConfiguration("camera_names").perform(context)
    image_topic = LaunchConfiguration("image_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    detections_topic = LaunchConfiguration("detections_topic").perform(context)

    rviz_config = LaunchConfiguration("rviz_config").perform(context)

    cams = [c.strip() for c in camera_names.split(",") if c.strip()]
    detection_topics = [f"/{c}/{detections_topic}" for c in cams]

    nodes = []

    # TF / robot description
    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", urdf_xacro]),
                value_type=str
            )
        }],
    ))

    # urdf_out = os.path.join("/tmp", "robot.urdf")

    # gen_urdf = ExecuteProcess(
    #     cmd=["bash", "-lc", f"xacro {urdf_xacro} > {urdf_out}"],
    #     output="screen",
    # )

    #     ExecuteProcess(
    #         cmd=[venv_python, "-m", "narwhal_robot.static_tf_node",
    #                         "--parent", "base_link",
    #                         "--cam1", "base_camera_1",
    #                         "--cam2", "base_camera_2",
    #                         "--cam3", "base_camera_3",
    #             ],
    #         output="screen",
    #     )

    # One apriltag node per camera namespace
    for cam in cams:
        nodes.append(Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            namespace=cam,
            output="screen",
            parameters=[{"image_transport": "raw"}],
            remappings=[
                ("image", image_topic),
                ("camera_info", camera_info_topic),
                ("detections", detections_topic),
            ],
        ))

    # apriltag_processes = []
    # for cam in cams:
    #     apriltag_processes.append(
    #         ExecuteProcess(
    #             cmd=["bash", "-lc",
    #                 " ".join([
    #                     "ros2 run apriltag_ros apriltag_node",
    #                     "--ros-args",
    #                     "-r", f"__ns:={cam}",
    #                     "-r", "__node:=apriltag",
    #                     "--param", "image_transport:=raw",
    #                     "-r", f"image:={image_topic}",
    #                     "-r", f"camera_info:={camera_info_topic}",
    #                     "-r", f"detections:={detections_topic}",
    #                 ])],
    #             output="screen",
    #         )
    #     )


    # Observer consumes all camera detections
    nodes.append(Node(
        package="tag_pose_observer",
        executable="tag_pose_observer",
        name="tag_pose_observer",
        output="screen",
        parameters=[
            observer_yaml,
            {
                "tag_map_yaml": tag_map_yaml,
                "detection_topics": detection_topics,
                "publish_topic": "/tag_global_pose",
            }
        ],
    ))

    # observer_runtime.yaml needs to be created with the following
    # tag_pose_observer:
    #     ros__parameters:
    #         tag_map_yaml: /path/to/tag_map.yaml
    #         detection_topics: ["/cam1/detections", "/cam2/detections"]
    #         publish_topic: /tag_global_pose

    # observer = ExecuteProcess(
    #     cmd=["bash", "-lc",
    #         f"ros2 run tag_pose_observer tag_pose_observer "
    #         f"--ros-args -r __node:=tag_pose_observer "
    #         f"--params-file {observer_yaml} "
    #         f"--params-file {observer_runtime_yaml}"],
    #     output="screen",
    # )


    # EKF
    nodes.append(Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_odom",
        output="screen",
        parameters=[ekf_odom_yaml],
    ))

    nodes.append(Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_map",
        output="screen",
        parameters=[ekf_map_yaml],
    ))

    # ekf = ExecuteProcess(
    #     cmd=["bash", "-lc",
    #         f"ros2 run robot_localization ekf_node "
    #         f"--ros-args -r __node:=ekf_map --params-file {ekf_map_yaml}"],
    #     output="screen",
    # )

    # RViz
    nodes.append(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    ))

    # rviz = ExecuteProcess(
    #     cmd=["bash", "-lc",
    #         f"ros2 run rviz2 rviz2 -- -d {rviz_config}"],
    #     output="screen",
    # )


    return nodes

def generate_launch_description():
    arena_description_share = get_package_share_directory("arena_description")
    arena_tag_map_share = get_package_share_directory("arena_tag_map")
    tag_pose_observer_share = get_package_share_directory("tag_pose_observer")
    localization_share = get_package_share_directory("localization_fusion")
    bringup_share = get_package_share_directory("arena_bringup")

    default_urdf_xacro = os.path.join(arena_description_share, "urdf", "robot.urdf.xacro")
    default_tag_map = os.path.join(arena_tag_map_share, "config", "arena_tags.yaml")
    default_observer_yaml = os.path.join(tag_pose_observer_share, "config", "observer.yaml")
    default_ekf_map_yaml = os.path.join(localization_share, "config", "ekf_map.yaml")
    default_ekf_odom_yaml = os.path.join(localization_share, "config", "ekf_odom.yaml")
    default_rviz = os.path.join(bringup_share, "config", "rviz.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("urdf_xacro", default_value=default_urdf_xacro),
        DeclareLaunchArgument("tag_map_yaml", default_value=default_tag_map),
        DeclareLaunchArgument("observer_yaml", default_value=default_observer_yaml),
        DeclareLaunchArgument("ekf_map_yaml", default_value=default_ekf_map_yaml),
        DeclareLaunchArgument("ekf_odom_yaml", default_value=default_ekf_odom_yaml),

        DeclareLaunchArgument("camera_names", default_value="camera_1,camera_2"),
        DeclareLaunchArgument("image_topic", default_value="image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="camera_info"),
        DeclareLaunchArgument("detections_topic", default_value="tag_detections"),

        DeclareLaunchArgument("rviz_config", default_value=default_rviz),

        OpaqueFunction(function=_make_nodes),
    ])
