import os, json, textwrap, zipfile, pathlib, math, datetime, shutil

src_json = "/mnt/data/2026-rebuilt-andymark.json"
with open(src_json, "r") as f:
    layout = json.load(f)

field_len = float(layout["field"]["length"])
field_wid = float(layout["field"]["width"])
tags = layout["tags"]

root = "/mnt/data/frc2026_ros2_localization_sim"
if os.path.exists(root):
    shutil.rmtree(root)
os.makedirs(root, exist_ok=True)

# --- Package 1: frc_apriltag_localization (Python) ---
pkg1 = os.path.join(root, "frc_apriltag_localization")
os.makedirs(os.path.join(pkg1, "frc_apriltag_localization"), exist_ok=True)
os.makedirs(os.path.join(pkg1, "launch"), exist_ok=True)
os.makedirs(os.path.join(pkg1, "config"), exist_ok=True)
os.makedirs(os.path.join(pkg1, "resource"), exist_ok=True)

# package.xml
package_xml = f"""<?xml version="1.0"?>
<package format="3">
  <name>frc_apriltag_localization</name>
  <version>0.1.0</version>
  <description>FRC 2026 AprilTag-based localization (tag map + vision pose) for ROS 2 Jazzy.</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>tf_transformations</exec_depend>
  <exec_depend>robot_localization</exec_depend>
  <exec_depend>apriltag_msgs</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
</package>
"""
open(os.path.join(pkg1, "package.xml"), "w").write(package_xml)

# setup.cfg + setup.py
setup_cfg = """[develop]
script_dir=$base/lib/frc_apriltag_localization
[install]
install_scripts=$base/lib/frc_apriltag_localization
"""
open(os.path.join(pkg1, "setup.cfg"), "w").write(setup_cfg)

setup_py = """from setuptools import setup
package_name = 'frc_apriltag_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/frc_localization.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf_odom.yaml', 'config/ekf_map.yaml']),
        ('share/' + package_name + '/config', ['config/tag_map_frc2026.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='FRC 2026 AprilTag-based localization for ROS 2 Jazzy.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'tag_localizer = frc_apriltag_localization.tag_localizer:main',
        ],
    },
)
"""
open(os.path.join(pkg1, "setup.py"), "w").write(setup_py)

# resource marker
open(os.path.join(pkg1, "resource", "frc_apriltag_localization"), "w").write("")

# Tag map JSON (store as-is + metadata)
tag_map = {
    "frame_id": "map",
    "field": {"length_m": field_len, "width_m": field_wid},
    "source": os.path.basename(src_json),
    "tags": tags
}
open(os.path.join(pkg1, "config", "tag_map_frc2026.json"), "w").write(json.dumps(tag_map, indent=2))

# EKF configs
ekf_odom_yaml = """ekf_odom:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # Inputs
    odom0: wheel/odometry
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_queue_size: 10
    odom0_nodelay: true

    imu0: imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  false, false, false]
    imu0_queue_size: 10
    imu0_nodelay: true
    imu0_remove_gravitational_acceleration: true

    # Noise (tune for your robot)
    process_noise_covariance: [0.05, 0,    0,    0,    0,    0,
                              0,    0.05, 0,    0,    0,    0,
                              0,    0,    1e6,  0,    0,    0,
                              0,    0,    0,    0.02, 0,    0,
                              0,    0,    0,    0,    0.02, 0,
                              0,    0,    0,    0,    0,    0.1]
"""
open(os.path.join(pkg1, "config", "ekf_odom.yaml"), "w").write(ekf_odom_yaml)

ekf_map_yaml = """ekf_map:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: map

    # Fuse the odom EKF output (provides smooth local odom)
    odom0: odometry/filtered
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_queue_size: 10
    odom0_nodelay: true

    # Vision global pose from tags
    pose0: vision/pose
    pose0_config: [true,  true,  false,
                   false, false, true,
                   false, false, false,
                   false, false, false,
                   false, false, false]
    pose0_queue_size: 10
    pose0_nodelay: true
    pose0_differential: false
    pose0_relative: false

    # Optional: reject bad tag updates when robot is spinning fast etc.
    # pose0_rejection_threshold: 2.5
"""
open(os.path.join(pkg1, "config", "ekf_map.yaml"), "w").write(ekf_map_yaml)

# tag_localizer.py
tag_localizer_py = r'''#!/usr/bin/env python3
import json
import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray

from tf_transformations import quaternion_matrix, quaternion_from_matrix
from tf2_ros import TransformBroadcaster


def _q_to_mat_xyzw(qx: float, qy: float, qz: float, qw: float):
    # 4x4 homogeneous rotation
    return quaternion_matrix([qx, qy, qz, qw])


def _make_T(translation_xyz, quat_xyzw):
    T = _q_to_mat_xyzw(*quat_xyzw)
    T[0, 3] = translation_xyz[0]
    T[1, 3] = translation_xyz[1]
    T[2, 3] = translation_xyz[2]
    return T


def _inv_T(T):
    # R^T and -R^T t
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    Ti = [[0.0]*4 for _ in range(4)]
    # fill
    for i in range(3):
        for j in range(3):
            Ti[i][j] = float(R[j, i])
    ti = [-sum(Ti[i][j]*t[j] for j in range(3)) for i in range(3)]
    for i in range(3):
        Ti[i][3] = float(ti[i])
    Ti[3] = [0.0, 0.0, 0.0, 1.0]
    import numpy as np
    return np.array(Ti, dtype=float)


@dataclass
class TagPose:
    T_map_tag: "object"  # numpy 4x4


class TagLocalizer(Node):
    """
    Consumes AprilTag detections (pose of tag in camera frame),
    uses a known tag map (pose of tag in map frame),
    and a known camera extrinsic (pose of camera in base_link frame),
    to compute base_link pose in map frame.
    """
    def __init__(self):
        super().__init__("frc_tag_localizer")

        self.declare_parameter("tag_map_json", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera")
        self.declare_parameter("detections_topic", "/tag_detections")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("tf_child_frame", "base_link")
        self.declare_parameter("vision_pose_topic", "vision/pose")

        # Camera extrinsic: T_base_cam (camera in base_link)
        self.declare_parameter("camera_translation", [0.0, 0.0, 0.0])  # meters
        self.declare_parameter("camera_rotation_xyzw", [0.0, 0.0, 0.0, 1.0])  # quaternion

        # Simple gating / fusion
        self.declare_parameter("min_detections", 1)
        self.declare_parameter("use_lowest_ambiguity", True)
        self.declare_parameter("pose_cov_xy_m2", 0.04)   # 0.2m std -> 0.04 variance
        self.declare_parameter("pose_cov_yaw_rad2", 0.09)  # 0.3rad std -> 0.09 variance

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        tag_map_path = self.get_parameter("tag_map_json").value
        if not tag_map_path:
            raise RuntimeError("tag_map_json parameter must be set to the WPILib-derived tag map JSON.")

        self.tag_poses: Dict[int, TagPose] = self._load_tag_map(tag_map_path)

        cam_t = self.get_parameter("camera_translation").value
        cam_q = self.get_parameter("camera_rotation_xyzw").value
        self.T_base_cam = _make_T(cam_t, cam_q)

        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter("vision_pose_topic").value, 10)

        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None

        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            self.get_parameter("detections_topic").value,
            self._on_detections,
            10
        )

        self.get_logger().info(f"Loaded {len(self.tag_poses)} tag poses; listening for detections on {self.get_parameter('detections_topic').value}")

    def _load_tag_map(self, path: str) -> Dict[int, TagPose]:
        import numpy as np
        with open(path, "r") as f:
            obj = json.load(f)

        poses: Dict[int, TagPose] = {}
        for t in obj["tags"]:
            tid = int(t["ID"])
            tr = t["pose"]["translation"]
            q = t["pose"]["rotation"]["quaternion"]
            # WPILib JSON stores quaternion keys X,Y,Z,W
            T = _make_T([float(tr["x"]), float(tr["y"]), float(tr["z"])],
                        [float(q["X"]), float(q["Y"]), float(q["Z"]), float(q["W"])])
            poses[tid] = TagPose(T_map_tag=np.array(T, dtype=float))
        return poses

    def _pick_detection(self, msg: AprilTagDetectionArray):
        # apriltag_msgs: each detection has id[] and pose (PoseWithCovarianceStamped-ish)
        # We use the first ID in detection.id
        if len(msg.detections) == 0:
            return None

        use_lowest = bool(self.get_parameter("use_lowest_ambiguity").value)
        if not use_lowest:
            return msg.detections[0]

        best = None
        best_amb = float("inf")
        for d in msg.detections:
            amb = getattr(d, "decision_margin", None)  # some builds
            if amb is None:
                amb = getattr(d, "pose", None)
            # If decision_margin doesn't exist, just take first
            if hasattr(d, "decision_margin"):
                # higher is better; invert
                score = -float(d.decision_margin)
            elif hasattr(d, "hamming"):
                score = float(d.hamming)
            else:
                score = 0.0
            if score < best_amb:
                best_amb = score
                best = d
        return best

    def _on_detections(self, msg: AprilTagDetectionArray):
        min_det = int(self.get_parameter("min_detections").value)
        if len(msg.detections) < min_det:
            return

        d = self._pick_detection(msg)
        if d is None:
            return

        tid = int(d.id[0]) if len(d.id) > 0 else -1
        if tid not in self.tag_poses:
            self.get_logger().debug(f"Unknown tag id {tid}")
            return

        # T_cam_tag from detection pose (tag in camera frame)
        p = d.pose.pose.pose.position
        o = d.pose.pose.pose.orientation
        T_cam_tag = _make_T([float(p.x), float(p.y), float(p.z)],
                            [float(o.x), float(o.y), float(o.z), float(o.w)])

        # Known: T_map_tag
        T_map_tag = self.tag_poses[tid].T_map_tag

        # base pose: T_map_base = T_map_tag * inv(T_cam_tag) * inv(T_base_cam)
        import numpy as np
        T_map_base = np.matmul(np.matmul(T_map_tag, _inv_T(T_cam_tag)), _inv_T(self.T_base_cam))

        # Extract translation + yaw (2D)
        x = float(T_map_base[0, 3])
        y = float(T_map_base[1, 3])
        yaw = math.atan2(float(T_map_base[1, 0]), float(T_map_base[0, 0]))

        # Build quaternion for yaw only
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        qx, qy, qz, qw = 0.0, 0.0, sy, cy

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        # Simple covariance (tune). Layout: row-major 6x6.
        cov_xy = float(self.get_parameter("pose_cov_xy_m2").value)
        cov_yaw = float(self.get_parameter("pose_cov_yaw_rad2").value)
        cov = [0.0]*36
        cov[0] = cov_xy
        cov[7] = cov_xy
        cov[35] = cov_yaw
        out.pose.covariance = cov

        self.pub_pose.publish(out)

        if self.publish_tf and self.tf_pub is not None:
            tfs = TransformStamped()
            tfs.header = out.header
            tfs.child_frame_id = self.get_parameter("tf_child_frame").value
            tfs.transform.translation.x = x
            tfs.transform.translation.y = y
            tfs.transform.translation.z = 0.0
            tfs.transform.rotation.x = qx
            tfs.transform.rotation.y = qy
            tfs.transform.rotation.z = qz
            tfs.transform.rotation.w = qw
            self.tf_pub.sendTransform(tfs)


def main():
    rclpy.init()
    node = TagLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
'''
open(os.path.join(pkg1, "frc_apriltag_localization", "tag_localizer.py"), "w").write(tag_localizer_py)

open(os.path.join(pkg1, "frc_apriltag_localization", "__init__.py"), "w").write("")

# Launch file
launch_py = r'''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("frc_apriltag_localization")
    tag_map = os.path.join(pkg_share, "config", "tag_map_frc2026.json")
    ekf_odom = os.path.join(pkg_share, "config", "ekf_odom.yaml")
    ekf_map  = os.path.join(pkg_share, "config", "ekf_map.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("detections_topic", default_value="/tag_detections"),
        DeclareLaunchArgument("camera_translation", default_value="[0.0, 0.0, 0.0]"),
        DeclareLaunchArgument("camera_rotation_xyzw", default_value="[0.0, 0.0, 0.0, 1.0]"),
        DeclareLaunchArgument("publish_tf", default_value="false"),

        # Tag localizer -> publishes vision/pose in map frame
        Node(
            package="frc_apriltag_localization",
            executable="tag_localizer",
            name="tag_localizer",
            output="screen",
            parameters=[{
                "tag_map_json": tag_map,
                "detections_topic": LaunchConfiguration("detections_topic"),
                "camera_translation": LaunchConfiguration("camera_translation"),
                "camera_rotation_xyzw": LaunchConfiguration("camera_rotation_xyzw"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "map_frame": "map",
                "base_frame": "base_link",
                "vision_pose_topic": "vision/pose",
            }]
        ),

        # Local odom EKF (wheel + IMU) publishes odometry/filtered and odom->base_link
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_odom",
            output="screen",
            parameters=[ekf_odom],
            remappings=[
                ("odometry/filtered", "odometry/filtered"),
            ],
        ),

        # Global map EKF (odom EKF + vision pose) publishes map->odom
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_map",
            output="screen",
            parameters=[ekf_map],
        ),
    ])
'''
open(os.path.join(pkg1, "launch", "frc_localization.launch.py"), "w").write(launch_py)

# --- Package 2: frc_field_sim (Gazebo Harmonic / gz sim) ---
pkg2 = os.path.join(root, "frc_field_sim")
os.makedirs(os.path.join(pkg2, "frc_field_sim"), exist_ok=True)
os.makedirs(os.path.join(pkg2, "launch"), exist_ok=True)
os.makedirs(os.path.join(pkg2, "worlds"), exist_ok=True)
os.makedirs(os.path.join(pkg2, "models", "frc_field", "materials", "textures"), exist_ok=True)
os.makedirs(os.path.join(pkg2, "scripts"), exist_ok=True)
os.makedirs(os.path.join(pkg2, "resource"), exist_ok=True)

open(os.path.join(pkg2, "resource", "frc_field_sim"), "w").write("")

package_xml2 = """<?xml version="1.0"?>
<package format="3">
  <name>frc_field_sim</name>
  <version>0.1.0</version>
  <description>Gazebo (gz sim) world for an FRC field with AprilTag visuals placed from WPILib layout JSON.</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
</package>
"""
open(os.path.join(pkg2, "package.xml"), "w").write(package_xml2)

setup_cfg2 = """[develop]
script_dir=$base/lib/frc_field_sim
[install]
install_scripts=$base/lib/frc_field_sim
"""
open(os.path.join(pkg2, "setup.cfg"), "w").write(setup_cfg2)

setup_py2 = """from setuptools import setup
package_name = 'frc_field_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/frc_field.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/frc2026_field.sdf']),
        ('share/' + package_name + '/scripts', ['scripts/generate_world_from_wpilib_json.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Gazebo world generator for FRC field with AprilTags.',
    license='BSD-3-Clause',
)
"""
open(os.path.join(pkg2, "setup.py"), "w").write(setup_py2)
open(os.path.join(pkg2, "frc_field_sim", "__init__.py"), "w").write("")

# World generator script
gen_script = r'''#!/usr/bin/env python3
"""
Generate an SDF world with AprilTag planes placed from a WPILib AprilTag layout JSON.

Usage:
  python3 generate_world_from_wpilib_json.py --input tag_layout.json --output frc_field.sdf

Notes:
- This generates *visuals only* (textures) for each tag. You must provide PNG textures yourself.
- Place textures under: models/frc_field/materials/textures/tag_<ID>.png
"""
import argparse, json, math

SDF_HEADER = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="frc_field">
    <gravity>0 0 -9.8</gravity>
"""

SDF_FOOTER = """  </world>
</sdf>
"""

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--tag_size_m", type=float, default=0.1651)  # black square size placeholder
    ap.add_argument("--thickness_m", type=float, default=0.005)
    args = ap.parse_args()

    with open(args.input, "r") as f:
        obj = json.load(f)

    tags = obj.get("tags", obj.get("Tags", []))
    field = obj.get("field", {})
    L = float(field.get("length", field.get("length_m", 16.518)))
    W = float(field.get("width", field.get("width_m", 8.043)))

    def tag_visual(tid, x, y, z, qx, qy, qz, qw):
        # Use a thin box; apply texture to all faces (good enough for visualization).
        # Pose uses quaternion; gz expects xyz + quaternion in <pose> if specified as: x y z roll pitch yaw.
        # We'll convert quaternion to rpy for SDF pose for simplicity.
        import numpy as np
        # quaternion -> yaw/pitch/roll (Z-Y-X) from matrix
        from math import atan2, asin
        # Build R
        # (minimal conversion)
        r11 = 1 - 2*(qy*qy + qz*qz)
        r21 = 2*(qx*qy + qz*qw)
        r31 = 2*(qx*qz - qy*qw)
        r32 = 2*(qy*qz + qx*qw)
        r33 = 1 - 2*(qx*qx + qy*qy)
        yaw = atan2(r21, r11)
        pitch = asin(max(-1.0, min(1.0, -r31)))
        roll = atan2(r32, r33)

        return f"""
    <model name="tag_{tid}">
      <static>true</static>
      <pose>{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>{args.tag_size_m:.6f} {args.tag_size_m:.6f} {args.thickness_m:.6f}</size></box>
          </geometry>
          <material>
            <pbr>
              <metal>
                <albedo_map>model://frc_field/materials/textures/tag_{tid}.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>
"""

    world = [SDF_HEADER]

    # Field plane (visual)
    world.append(f"""
    <model name="field_plane">
      <static>true</static>
      <pose>{L/2:.6f} {W/2:.6f} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{L:.6f} {W:.6f} 0.05</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{L:.6f} {W:.6f} 0.02</size></box>
          </geometry>
        </visual>
      </link>
    </model>
""")

    for t in tags:
        tid = int(t["ID"])
        tr = t["pose"]["translation"]
        q = t["pose"]["rotation"]["quaternion"]
        x = float(tr["x"]); y = float(tr["y"]); z = float(tr["z"])
        qx = float(q["X"]); qy = float(q["Y"]); qz = float(q["Z"]); qw = float(q["W"])
        world.append(tag_visual(tid, x, y, z, qx, qy, qz, qw))

    world.append(SDF_FOOTER)
    with open(args.output, "w") as f:
        f.write("".join(world))

if __name__ == "__main__":
    main()
'''
open(os.path.join(pkg2, "scripts", "generate_world_from_wpilib_json.py"), "w").write(gen_script)

# Pre-generate world SDF using the user's JSON
world_sdf_path = os.path.join(pkg2, "worlds", "frc2026_field.sdf")
# reuse generator logic quickly inline:
def quat_to_rpy(qx,qy,qz,qw):
    r11 = 1 - 2*(qy*qy + qz*qz)
    r21 = 2*(qx*qy + qz*qw)
    r31 = 2*(qx*qz - qy*qw)
    r32 = 2*(qy*qz + qx*qw)
    r33 = 1 - 2*(qx*qx + qy*qy)
    yaw = math.atan2(r21, r11)
    pitch = math.asin(max(-1.0, min(1.0, -r31)))
    roll = math.atan2(r32, r33)
    return roll,pitch,yaw

tag_size_m = 0.1651
thickness_m = 0.005

parts = ["""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="frc_field">
    <gravity>0 0 -9.8</gravity>
"""]
parts.append(f"""
    <model name="field_plane">
      <static>true</static>
      <pose>{field_len/2:.6f} {field_wid/2:.6f} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{field_len:.6f} {field_wid:.6f} 0.05</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{field_len:.6f} {field_wid:.6f} 0.02</size></box>
          </geometry>
        </visual>
      </link>
    </model>
""")
for t in tags:
    tid=int(t["ID"])
    tr=t["pose"]["translation"]; q=t["pose"]["rotation"]["quaternion"]
    x=float(tr["x"]); y=float(tr["y"]); z=float(tr["z"])
    qx=float(q["X"]); qy=float(q["Y"]); qz=float(q["Z"]); qw=float(q["W"])
    roll,pitch,yaw=quat_to_rpy(qx,qy,qz,qw)
    parts.append(f"""
    <model name="tag_{tid}">
      <static>true</static>
      <pose>{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>{tag_size_m:.6f} {tag_size_m:.6f} {thickness_m:.6f}</size></box>
          </geometry>
          <material>
            <pbr>
              <metal>
                <albedo_map>model://frc_field/materials/textures/tag_{tid}.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>
""")
parts.append("""  </world>
</sdf>
""")
open(world_sdf_path,"w").write("".join(parts))

# Launch file for gz sim
launch_sim = r'''from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("frc_field_sim")
    world = os.path.join(pkg, "worlds", "frc2026_field.sdf")

    return LaunchDescription([
        Node(
            package="ros_gz_sim",
            executable="gz_sim",
            output="screen",
            arguments=["-r", world],
        ),
    ])
'''
open(os.path.join(pkg2, "launch", "frc_field.launch.py"), "w").write(launch_sim)

# README with run steps
readme = f"""# FRC 2026 ROS 2 Localization + Sim (Jazzy)

This repo contains two ROS 2 packages:

- `frc_apriltag_localization`: Python AprilTag -> global pose node + `robot_localization` EKF configs (competition-grade pattern).
- `frc_field_sim`: `gz sim` (Gazebo Harmonic) world with AprilTag visuals placed from the official WPILib layout JSON.

## 1) Build (colcon)
```bash
mkdir -p ~/ws_frc/src
cp -r frc_apriltag_localization frc_field_sim ~/ws_frc/src/
cd ~/ws_frc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
