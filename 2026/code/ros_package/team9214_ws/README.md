# team9214 ROS2 Package

## Directory Layout

```bash
.
├── arena_bringup   # “one command” launch for everything
│   ├── arena_bringup   # Mostly NT4 to ROS2 coordination nodes
│   │   ├── autonomy_mode_manager.py   
│   │   ├── cmd_vel_nt4_bridge.py         
│   │   ├── nt4_mode_bridge.py            
│   │   └── sim_inputs.py                 
│   ├── config  # Full system config for ROS2 package launching
│   │   ├── bringup.yaml  # paths, camera list, topic names                      
│   │   ├── observer_runtime.yaml               
│   │   └── rviz.rviz                           
│   ├── docs    # Most diagrams
│   │   ├── bringup_multi_cam_nodes_topics.puml 
│   │   ├── bringup_multi_cam_nodes_topics.svg
│   │   ├── bringup_nodes_topics.puml
│   │   ├── bringup_nodes_topics.svg
│   │   ├── bringup_single_cam_nodes_topics.puml
│   │   ├── bringup_single_cam_nodes_topics.svg
│   │   ├── docker-compose.yaml
│   │   └── README.md
│   ├── launch  # Main system launch files
│   │   ├── bringup_multi_cam.launch.py
│   │   ├── bringup_single_cam.launch.py
│   │   ├── localization.launch.py
│   │   ├── navigation.launch.py
│   │   └── sim_harness.launch.py
│   ├── package.xml     # Standard ROS2 package
│   ├── params  # Addition parameter files
│   │   └── nav2_params.yaml
│   ├── README.md   # Overview of arena_bringup
│   ├── resource
│   │   └── arena_bringup
│   ├── setup.cfg
│   └── setup.py
├── arena_description   # URDF/Xacro + meshes
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   └── urdf
│       ├── robot.urdf copy.xacro
│       └── robot.urdf.xacro    # Robot transforms description
# arena_tag_map package
# Map frame conventions (implicitly by how place pose)
# Tag id/family/size
# tag layout - each tags pose in the map
├── arena_tag_map   # arena coordinate frame + tag layout + helper library
│   ├── CMakeLists.txt
│   ├── config  # tag layout files - minimum viable “arena map” for tag localization
│   │   ├── 757-lab_arena_tags.yaml
│   │   ├── arena_tags.yaml
│   │   ├── camera_test_arena_tags.yaml
│   │   ├── example_arena_tags.yaml
│   │   ├── frc2026_apriltag_ros.yaml
│   │   ├── frc2026_field_blank.pgm
│   │   ├── frc2026_field_blank.yaml
│   │   └── frc2026_tag_static_tfs.yaml
│   ├── package.xml
│   ├── README.md
│   └── scripts
│       └── load_map_in_nav2.sh
├── localization_fusion # robot_localization config + launch
│   ├── CMakeLists.txt
│   ├── config
│   │   └── ekf_map.yaml    # global updates -> map->odom
│   ├── launch
│   │   └── fusion.launch.py
│   └── package.xml
├── multi_cam_apriltag  # Camera apriltag pipeline package
│   ├── config              
│   │   ├── camera_1.yaml                   # calibration file                
│   │   ├── camera_2.yaml
│   │   └── usb_cam.yaml                    # camera config file
│   └── launch
│       └── single-cam-apriltag_node.launch.py
├── tag_pose_observer           # converts detections -> map-frame base pose observations
│   ├── CMakeLists.txt
│   ├── config
│   │   └── observer.yaml       # T_map_base from T_map_tag, T_cam_tag, T_base_cam
│   ├── launch
│   │   └── observer.launch.py
│   ├── package.xml
│   ├── README.md
│   ├── resource
│   │   └── tag_pose_observer
│   ├── setup.cfg
│   ├── setup.py
│   └── tag_pose_observer
│       ├── __init__.py
│       └── observer_node.py
```

## Build package

`colcon build --packages-up-to arena_bringup --event-handlers console_direct+`

## Build Troubleshooting

If `colcon build --symlink-install` fails with errors like:
- `ModuleNotFoundError: No module named 'ament_package'`
- `Could not find a package configuration file provided by "ament_cmake"`

run this recovery sequence from a fresh terminal:

```bash
cd ~/source_code/first_robotics_comp/frc/team9214_ws

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
```

Optional: auto-source ROS in new shells by adding this to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
```
