from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    pkg_share = FindPackageShare("honking_narwhal")  # <-- your package name that contains config/
    
    usb_cam_params = PathJoinSubstitution([pkg_share, "config", "usb_cam.yaml"])
    apriltag_params = PathJoinSubstitution([pkg_share, "config", "apriltag.yaml"])

    #     ExecuteProcess(
    #         cmd=[venv_python, "-m", "narwhal_robot.static_tf_node",
    #                         "--parent", "base_link",
    #                         "--cam1", "base_camera_1",
    #                         "--cam2", "base_camera_2",
    #                         "--cam3", "base_camera_3",
    #             ],
    #         output="screen",
    #     )
    
    usb_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        # Fully qualified namespace /camera/usb_cam.
        # Param file needs to be under this fqns
        name="usb_cam",
        namespace="camera",
        # parameters=[usb_cam_params],
        parameters=["/home/robots/source_code/first_robotics_comp/frc/camera_ws/src/honking_narwhal/config/usb_cam.yaml"],
        output="screen",
    )
 
    # Rectify node: subscribes to image_raw + 
    # camera_info, publishes image_rect. 
    # :contentReference[oaicite:2]{index=2}
    rectify = Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_node",
        namespace="camera",
        remappings=[
            ("image", "image_raw"),
            ("camera_info", "camera_info"),
            ("image_rect", "image_rect"),
        ],
        output="screen",
    )
 
    # AprilTag node subscribes to image_rect + 
    # camera_info; publishes detections + tf. 
    # :contentReference[oaicite:3]{index=3}
    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        namespace="camera",
        parameters=["/home/robots/source_code/first_robotics_comp/frc/camera_ws/src/honking_narwhal/config/apriltag.yaml"],
        remappings=[
            ("image_rect", "image_rect"),
            ("camera_info", "camera_info"),
        ],
        output="screen",
    )

    # classifier = Node(
    #     package="tag_classifier",
    #     executable="tag_classifier",
    #     name="tag_classifier",
    #     namespace="camera",
    #     parameters=[{
    #         "tag_map_yaml": "/work/config/tag_map.yaml",
    #         "detections_topic": "/camera/detections",
    #         "publish_topic": "/camera/tag_classification",
    #     }],
    #     output="screen",
    # )

    # return LaunchDescription([usb_cam, rectify, apriltag, classifier])

    return LaunchDescription([usb_cam, rectify, apriltag])