# team9214 ROS2 Package

## Core Pipeline: usb_cam -> rectifier -> tag detector -> position

This repo focuses on the essential AprilTag-based localization pipeline:

1. **usb_cam**: Captures camera images
2. **image_proc/rectify**: Rectifies images using camera calibration
3. **apriltag_ros**: Detects AprilTags in images
4. **tag_pose_observer**: Computes robot pose from tag detections

## Directory Layout

```bash
.
├── arena_bringup   # Launch files and configs for the core pipeline
│   ├── config
│   │   ├── bringup.yaml  # Main config file
│   │   └── rviz.rviz     # RViz visualization config
│   └── launch
│       ├── bringup_single_cam.launch.py  # Main launch file
│       └── ...
├── arena_robot_and_maps  # Robot and map definitions
│   ├── arena_description   # Robot URDF/Xacro
│   └── arena_tag_map       # AprilTag map definitions
│       └── config
│           ├── camera_test_arena_tags.yaml  # Tag poses in map frame
│           └── frc2026_apriltag_ros.yaml    # AprilTag detection params
└── tag_pose_observer   # Pose estimation from tag detections
    └── config
        └── observer.yaml  # Observer parameters
```

## Quick Start

1. Configure camera and tag map in `arena_bringup/config/bringup.yaml`
2. Launch the pipeline:
   ```bash
   ros2 launch arena_bringup bringup_single_cam.launch.py
   ```
3. View in RViz or check `/tag_global_pose` topic

## Build

```bash
colcon build --packages-select arena_bringup arena_description arena_tag_map tag_pose_observer
```

## Build Troubleshooting

If `colcon build --symlink-install` fails with errors like:
- `ModuleNotFoundError: No module named 'ament_package'`
- `Could not find a package configuration file provided by "ament_cmake"`

run this recovery sequence from a fresh terminal:

```bash
cd ~/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws

# clear stale CMake cache from failed builds
rm -rf build install log

# avoid venv/conda path conflicts during ROS builds
deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true

# source ROS Jazzy environment
source /opt/ros/jazzy/setup.bash

# quick checks
echo "$ROS_DISTRO"
echo "$AMENT_PREFIX_PATH"
echo "$CMAKE_PREFIX_PATH" | grep -q "/opt/ros/jazzy" && echo "ROS paths OK"

colcon build --symlink-install
```

Install prerequisites (if needed):

```bash
sudo apt update
sudo apt install ros-jazzy-ament-cmake ros-jazzy-ament-package
sudo apt install ros-jazzy-rosbag2-storage-mcap
```

Optional: auto-source ROS in new shells by adding this to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
```

## Logging ROS2 Data

# good: explicit topics
`ros2 bag record /camera_1/image_raw /camera_1/camera_info /camera_1/image_rect /camera_1/tag_detections`

# or if recording all, exclude depth transport topics
`ros2 bag record -a --exclude ".*compressedDepth.*"`

`ros2 topic info /camera_1/image_raw/compressedDepth -v`