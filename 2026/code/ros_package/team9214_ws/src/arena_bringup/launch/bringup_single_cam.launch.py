"""
Single-camera full-system bringup.

Default values are loaded from a YAML file (bringup.yaml). CLI launch arguments
still work and override YAML values.
"""

import os
import subprocess
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


AUTO = "__AUTO__"


def _load_bringup_params(bringup_config_path: str) -> dict:
    try:
        with open(bringup_config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}

    return data.get("bringup", {}).get("ros__parameters", {})


def _load_apriltag_params(apriltag_params_path: str) -> dict:
    if not apriltag_params_path:
        return {}
    try:
        with open(apriltag_params_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}

    if not isinstance(data, dict):
        return {}

    # Accept common ROS2 param file layouts:
    # - apriltag_ros: { ros__parameters: {...} }
    # - apriltag: { ros__parameters: {...} }
    # - ros__parameters: {...}
    def sanitize(params: dict) -> dict:
        # launch_ros parameter normalization does not support nested dicts/lists of dicts.
        # Keep only scalar params and lists of scalars.
        def is_scalar(v):
            return isinstance(v, (bool, int, float, str))

        out = {}
        for k, v in params.items():
            if is_scalar(v):
                out[k] = v
            elif isinstance(v, list) and all(is_scalar(x) for x in v):
                out[k] = v
        return out

    for key in ("apriltag_ros", "apriltag"):
        node_block = data.get(key, {})
        if isinstance(node_block, dict):
            ros_params = node_block.get("ros__parameters", {})
            if isinstance(ros_params, dict):
                return sanitize(ros_params)

    ros_params = data.get("ros__parameters", {})
    if isinstance(ros_params, dict):
        return sanitize(ros_params)

    return {}


def _prepare_ekf_params_yaml(
    src_yaml_path: str,
    node_name: str,
    use_wheel_odom: bool,
    use_imu: bool,
    world_frame_override: str = "",
    odom_frame_override: str = "",
) -> str:
    with open(src_yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    node_block = data.get(node_name, {})
    ros_params = node_block.get("ros__parameters", {}) if isinstance(node_block, dict) else {}
    if not isinstance(ros_params, dict):
        ros_params = {}

    if not use_wheel_odom:
        for key in ("odom0", "odom0_config", "odom0_queue_size", "odom0_differential", "odom0_relative"):
            ros_params.pop(key, None)

    if not use_imu:
        for key in ("imu0", "imu0_config", "imu0_queue_size", "imu0_differential", "imu0_relative"):
            ros_params.pop(key, None)

    if world_frame_override:
        ros_params["world_frame"] = world_frame_override
    if odom_frame_override:
        ros_params["odom_frame"] = odom_frame_override

    data[node_name] = {"ros__parameters": ros_params}

    fd, temp_path = tempfile.mkstemp(prefix=f"{node_name}_", suffix=".yaml")
    os.close(fd)
    with open(temp_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)

    return temp_path


def _resolve_value(name: str, raw_value: str, cfg: dict, fallback: str) -> str:
    if raw_value not in ("", AUTO):
        return raw_value

    cfg_value = cfg.get(name, "")
    if cfg_value in (None, ""):
        return fallback

    return str(cfg_value)


def _normalize_camera_info_url(value: str) -> str:
    # usb_cam expects a URL (for local files: file:///abs/path).
    if not value:
        return value
    if "://" in value:
        return value
    if os.path.isabs(value):
        return f"file://{value}"
    return value


def _to_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _load_robot_description(urdf_xacro_path: str) -> str:
    # Prefer xacro expansion. If this file is plain URDF-with-.xacro extension,
    # a direct read fallback keeps bringup working.
    try:
        return subprocess.check_output(["xacro", urdf_xacro_path], text=True)
    except Exception:
        with open(urdf_xacro_path, "r", encoding="utf-8") as f:
            return f.read()


def _make_nodes(context, *args, **kwargs):
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
    default_apriltag_params_yaml = os.path.join(arena_tag_map_share, "config", "frc2026_apriltag_ros.yaml")
    default_camera_info_url = (
        "file:///home/robots/source_code/first_robotics_comp/frc/"
        "team9214_ws/src/multi_cam_apriltag/config/camera_1.yaml"
    )

    bringup_config = LaunchConfiguration("bringup_config").perform(context)
    cfg = _load_bringup_params(bringup_config)

    urdf_xacro = _resolve_value(
        "urdf_xacro",
        LaunchConfiguration("urdf_xacro").perform(context),
        cfg,
        default_urdf_xacro,
    )
    observer_yaml = _resolve_value(
        "observer_yaml",
        LaunchConfiguration("observer_yaml").perform(context),
        cfg,
        default_observer_yaml,
    )
    tag_map_yaml = _resolve_value(
        "tag_map_yaml",
        LaunchConfiguration("tag_map_yaml").perform(context),
        cfg,
        default_tag_map,
    )
    ekf_map_yaml = _resolve_value(
        "ekf_map_yaml",
        LaunchConfiguration("ekf_map_yaml").perform(context),
        cfg,
        default_ekf_map_yaml,
    )
    ekf_odom_yaml = _resolve_value(
        "ekf_odom_yaml",
        LaunchConfiguration("ekf_odom_yaml").perform(context),
        cfg,
        default_ekf_odom_yaml,
    )
    rviz_config = _resolve_value(
        "rviz_config",
        LaunchConfiguration("rviz_config").perform(context),
        cfg,
        default_rviz,
    )
    apriltag_params_yaml = _resolve_value(
        "apriltag_params_yaml",
        LaunchConfiguration("apriltag_params_yaml").perform(context),
        cfg,
        default_apriltag_params_yaml,
    )

    camera_ns = _resolve_value(
        "camera_ns",
        LaunchConfiguration("camera_ns").perform(context),
        cfg,
        "camera_1",
    )
    image_topic = _resolve_value(
        "image_topic",
        LaunchConfiguration("image_topic").perform(context),
        cfg,
        "image_rect",
    )
    camera_info_topic = _resolve_value(
        "camera_info_topic",
        LaunchConfiguration("camera_info_topic").perform(context),
        cfg,
        "camera_info",
    )
    detections_topic = _resolve_value(
        "detections_topic",
        LaunchConfiguration("detections_topic").perform(context),
        cfg,
        "tag_detections",
    )

    use_rviz = _to_bool(
        _resolve_value("use_rviz", LaunchConfiguration("use_rviz").perform(context), cfg, "true")
    )
    use_camera_driver = _to_bool(
        _resolve_value(
            "use_camera_driver",
            LaunchConfiguration("use_camera_driver").perform(context),
            cfg,
            "true",
        )
    )
    use_rectify = _to_bool(
        _resolve_value(
            "use_rectify",
            LaunchConfiguration("use_rectify").perform(context),
            cfg,
            "true",
        )
    )
    use_static_camera_tf = _to_bool(
        _resolve_value(
            "use_static_camera_tf",
            LaunchConfiguration("use_static_camera_tf").perform(context),
            cfg,
            "true",
        )
    )
    use_wheel_odom = _to_bool(
        _resolve_value(
            "use_wheel_odom",
            LaunchConfiguration("use_wheel_odom").perform(context),
            cfg,
            "true",
        )
    )
    use_imu = _to_bool(
        _resolve_value(
            "use_imu",
            LaunchConfiguration("use_imu").perform(context),
            cfg,
            "true",
        )
    )
    run_ekf_odom = _to_bool(
        _resolve_value(
            "run_ekf_odom",
            LaunchConfiguration("run_ekf_odom").perform(context),
            cfg,
            "true",
        )
    )

    video_device = _resolve_value(
        "video_device",
        LaunchConfiguration("video_device").perform(context),
        cfg,
        "/dev/video0",
    )
    framerate = float(
        _resolve_value(
            "framerate",
            LaunchConfiguration("framerate").perform(context),
            cfg,
            "30.0",
        )
    )
    image_width = int(
        _resolve_value(
            "image_width",
            LaunchConfiguration("image_width").perform(context),
            cfg,
            "640",
        )
    )
    image_height = int(
        _resolve_value(
            "image_height",
            LaunchConfiguration("image_height").perform(context),
            cfg,
            "480",
        )
    )
    pixel_format = _resolve_value(
        "pixel_format",
        LaunchConfiguration("pixel_format").perform(context),
        cfg,
        "yuyv",
    )
    io_method = _resolve_value(
        "io_method",
        LaunchConfiguration("io_method").perform(context),
        cfg,
        "mmap",
    )
    camera_frame = _resolve_value(
        "camera_frame",
        LaunchConfiguration("camera_frame").perform(context),
        cfg,
        "camera_1_optical_frame",
    )
    camera_info_url = _resolve_value(
        "camera_info_url",
        LaunchConfiguration("camera_info_url").perform(context),
        cfg,
        default_camera_info_url,
    )
    camera_info_url = _normalize_camera_info_url(camera_info_url)
    camera_tf_x = _resolve_value(
        "camera_tf_x",
        LaunchConfiguration("camera_tf_x").perform(context),
        cfg,
        "0.20",
    )
    camera_tf_y = _resolve_value(
        "camera_tf_y",
        LaunchConfiguration("camera_tf_y").perform(context),
        cfg,
        "0.00",
    )
    camera_tf_z = _resolve_value(
        "camera_tf_z",
        LaunchConfiguration("camera_tf_z").perform(context),
        cfg,
        "0.30",
    )
    camera_tf_roll = _resolve_value(
        "camera_tf_roll",
        LaunchConfiguration("camera_tf_roll").perform(context),
        cfg,
        "-1.57079632679",
    )
    camera_tf_pitch = _resolve_value(
        "camera_tf_pitch",
        LaunchConfiguration("camera_tf_pitch").perform(context),
        cfg,
        "0.0",
    )
    camera_tf_yaw = _resolve_value(
        "camera_tf_yaw",
        LaunchConfiguration("camera_tf_yaw").perform(context),
        cfg,
        "-1.57079632679",
    )

    for name, path in (
        ("urdf_xacro", urdf_xacro),
        ("tag_map_yaml", tag_map_yaml),
        ("observer_yaml", observer_yaml),
        ("ekf_map_yaml", ekf_map_yaml),
        ("ekf_odom_yaml", ekf_odom_yaml),
        ("rviz_config", rviz_config),
        ("apriltag_params_yaml", apriltag_params_yaml),
    ):
        if not path:
            raise RuntimeError(f"{name} resolved to empty path; set it in bringup.yaml or via launch arg")
        if not os.path.isfile(path):
            raise RuntimeError(
                f"{name} does not exist: {path}. "
                f"Set it in bringup.yaml or pass {name}:=/absolute/path"
            )

    detection_topic = f"/{camera_ns}/{detections_topic}"
    robot_description = _load_robot_description(urdf_xacro)
    apriltag_params = _load_apriltag_params(apriltag_params_yaml)
    # In single-filter mode (run_ekf_odom=false), keep frames unique:
    # map_frame=map, odom_frame=odom, base_link_frame=base_link.
    # Use world_frame=odom so ekf_map publishes odom->base_link directly.
    ekf_world_frame_override = "odom" if not run_ekf_odom else ""
    ekf_odom_frame_override = ""
    ekf_map_params_yaml = _prepare_ekf_params_yaml(
        ekf_map_yaml,
        "ekf_map",
        use_wheel_odom=use_wheel_odom,
        use_imu=use_imu,
        world_frame_override=ekf_world_frame_override,
        odom_frame_override=ekf_odom_frame_override,
    )
    ekf_odom_params_yaml = _prepare_ekf_params_yaml(
        ekf_odom_yaml,
        "ekf_odom",
        use_wheel_odom=use_wheel_odom,
        use_imu=use_imu,
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
    ]

    if use_camera_driver:
        nodes.append(
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam",
                namespace=camera_ns,
                output="screen",
                parameters=[{
                    "video_device": video_device,
                    "framerate": framerate,
                    "image_width": image_width,
                    "image_height": image_height,
                    "pixel_format": pixel_format,
                    "io_method": io_method,
                    "camera_name": camera_ns,
                    "frame_id": camera_frame,
                    "camera_info_url": camera_info_url,
                }],
            )
        )

    if use_static_camera_tf:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera_static_tf",
                output="screen",
                arguments=[
                    "--x", camera_tf_x,
                    "--y", camera_tf_y,
                    "--z", camera_tf_z,
                    "--roll", camera_tf_roll,
                    "--pitch", camera_tf_pitch,
                    "--yaw", camera_tf_yaw,
                    "--frame-id", "base_link",
                    "--child-frame-id", camera_frame,
                ],
            )
        )

    if use_rectify:
        nodes.append(
            Node(
                package="image_proc",
                executable="rectify_node",
                name="rectify_node",
                namespace=camera_ns,
                output="screen",
                remappings=[
                    ("image", "image_raw"),
                    ("camera_info", "camera_info"),
                    ("image_rect", "image_rect"),
                ],
            )
        )

    nodes.extend([
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            namespace=camera_ns,
            output="screen",
            parameters=[apriltag_params, {"image_transport": "raw"}],
            remappings=[
                ("image", image_topic),
                ("camera_info", camera_info_topic),
                ("detections", detections_topic),
            ],
        ),
        Node(
            package="tag_pose_observer",
            executable="tag_pose_observer",
            name="tag_pose_observer",
            output="screen",
            parameters=[
                observer_yaml,
                {
                    "tag_map_yaml": tag_map_yaml,
                    "detection_topics": [detection_topic],
                    "camera_frame": camera_frame,
                    "publish_topic": "/tag_global_pose",
                },
            ],
        ),
    ])

    if run_ekf_odom:
        nodes.append(
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_odom",
                output="screen",
                parameters=[ekf_odom_params_yaml],
            )
        )

    nodes.append(
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_map",
            output="screen",
            parameters=[ekf_map_params_yaml],
        )
    )

    if use_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            )
        )

    return nodes


def generate_launch_description():
    bringup_share = get_package_share_directory("arena_bringup")
    default_bringup_config = os.path.join(bringup_share, "config", "bringup.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("bringup_config", default_value=default_bringup_config),

        # AUTO means "read from bringup.yaml, else fallback internal default"
        DeclareLaunchArgument("urdf_xacro", default_value=AUTO),
        DeclareLaunchArgument("tag_map_yaml", default_value=AUTO),
        DeclareLaunchArgument("observer_yaml", default_value=AUTO),
        DeclareLaunchArgument("ekf_map_yaml", default_value=AUTO),
        DeclareLaunchArgument("ekf_odom_yaml", default_value=AUTO),
        DeclareLaunchArgument("rviz_config", default_value=AUTO),
        DeclareLaunchArgument("apriltag_params_yaml", default_value=AUTO),
        DeclareLaunchArgument("use_rviz", default_value=AUTO),

        DeclareLaunchArgument("camera_ns", default_value=AUTO),
        DeclareLaunchArgument("image_topic", default_value=AUTO),
        DeclareLaunchArgument("camera_info_topic", default_value=AUTO),
        DeclareLaunchArgument("detections_topic", default_value=AUTO),

        DeclareLaunchArgument("use_camera_driver", default_value=AUTO),
        DeclareLaunchArgument("use_rectify", default_value=AUTO),
        DeclareLaunchArgument("video_device", default_value=AUTO),
        DeclareLaunchArgument("framerate", default_value=AUTO),
        DeclareLaunchArgument("image_width", default_value=AUTO),
        DeclareLaunchArgument("image_height", default_value=AUTO),
        DeclareLaunchArgument("pixel_format", default_value=AUTO),
        DeclareLaunchArgument("io_method", default_value=AUTO),
        DeclareLaunchArgument("camera_frame", default_value=AUTO),
        DeclareLaunchArgument("camera_info_url", default_value=AUTO),
        DeclareLaunchArgument("use_static_camera_tf", default_value=AUTO),
        DeclareLaunchArgument("use_wheel_odom", default_value=AUTO),
        DeclareLaunchArgument("use_imu", default_value=AUTO),
        DeclareLaunchArgument("run_ekf_odom", default_value=AUTO),
        DeclareLaunchArgument("camera_tf_x", default_value=AUTO),
        DeclareLaunchArgument("camera_tf_y", default_value=AUTO),
        DeclareLaunchArgument("camera_tf_z", default_value=AUTO),
        DeclareLaunchArgument("camera_tf_roll", default_value=AUTO),
        DeclareLaunchArgument("camera_tf_pitch", default_value=AUTO),
        DeclareLaunchArgument("camera_tf_yaw", default_value=AUTO),

        OpaqueFunction(function=_make_nodes),
    ])
