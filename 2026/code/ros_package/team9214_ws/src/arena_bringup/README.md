# arena_bringup

Bringup package for localization + Nav2 + behavior-tree autonomy in Team 9214 ROS2.

## What this package provides

Python nodes:
- `autonomy_mode_manager.py`: sends/cancels `NavigateToPose` based on mode transitions.
- `cmd_vel_nt4_bridge.py`: publishes Nav2 velocity/status to NT4.
- `nt4_mode_bridge.py`: reads robot mode from NT4 and republishes to ROS.
- `sim_inputs.py`: simulation helper publishing mode, odom, TF, and tag pose.

Launch files:
- `bringup_single_cam.launch.py`: single-camera localization pipeline.
- `bringup_multi_cam.launch.py`: multi-camera variant.
- `navigation.launch.py`: Nav2 stack + BT navigator (+ optional map server + bridges).
- `sim_harness.launch.py`: sim helper + optional navigation stack.

## Build

From workspace root:

```bash
cd /home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws
colcon build --symlink-install --packages-up-to arena_bringup
source install/setup.bash
```

## Run: single-camera localization

```bash
ros2 launch arena_bringup bringup_single_cam.launch.py \
  camera_ns:=camera_1 \
  tag_map_yaml:=/home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_tag_map/config/757-lab_arena_tags.yaml
```

## Run: navigation stack directly

```bash
ros2 launch arena_bringup navigation.launch.py \
  params_file:=/home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
  launch_map_server:=true \
  map:=/home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_tag_map/config/frc2026_field_blank.yaml \
  autostart:=true
```

## Run: full sim harness (recommended for BT/Nav2 testing)

Current working defaults in `sim_harness.launch.py`:
- map server enabled (`nav_launch_map_server:=true`)
- sim params file (`nav2_params_sim.yaml`)
- keepout/speed zones disabled by default in sim
- start pose inside map (`sim_start_x:=1.0`, `sim_start_y:=1.0`)
- autonomy manager defaults to continuous loop mode (`autonomy_goal_mode:=loop_waypoints`)
- loop route defaults to compact rectangle:
  `1.0,1.0,0.0;3.0,1.0,0.0;3.0,2.0,1.57;1.0,2.0,3.14`
- `sim_inputs` follows `/cmd_vel_nav` so controller progress works

Launch:

```bash
ros2 launch arena_bringup sim_harness.launch.py \
  launch_navigation:=true \
  nav_params_file:=/home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_bringup/params/nav2_params_sim.yaml \
  nav_autostart:=true \
  nav_launch_map_server:=true \
  nav_map_yaml:=/home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_tag_map/config/frc2026_field_blank.yaml \
  enable_autonomy_mode_manager:=true \
  autonomy_goal_behavior_tree:=/home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_bringup/behavior_trees/custom_bt.xml \
  mode_schedule:=disabled:3,teleop:3,autonomous:12,disabled:3
```

Useful overrides:
- `use_keepout_zones:=true|false`
- `use_speed_zones:=true|false`
- `sim_start_x:=<m> sim_start_y:=<m> sim_start_yaw:=<rad>`
- `autonomy_goal_x:=<m> autonomy_goal_y:=<m> autonomy_goal_yaw:=<rad>`
- `autonomy_goal_mode:=single_goal|loop_waypoints`
- `autonomy_waypoints:=x,y,yaw;x,y,yaw;...`
- `autonomy_loop_on_success:=true|false`
- `autonomy_skip_failed_waypoint:=true|false`
- `launch_navigation:=false` (run `sim_inputs` only)

## Observe behavior

Terminal 2:

```bash
source /home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/install/setup.bash
ros2 topic echo /navigate_to_pose/_action/status
```

Terminal 3:

```bash
source /home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/install/setup.bash
ros2 topic echo /robot_mode
```

Force one autonomous transition:

```bash
ros2 topic pub -1 /robot_mode std_msgs/msg/String "{data: autonomous}"
```

## Continuous autonomous loop

`autonomy_mode_manager` supports two modes:
- `single_goal`: sends one `NavigateToPose` goal (existing behavior)
- `loop_waypoints`: continuously cycles waypoint goals while mode remains autonomous

Waypoint format:
- `x,y,yaw;x,y,yaw;...` in `goal_frame_id` (default `map`)
- Example:
  `1.0,1.0,0.0;3.0,1.0,0.0;3.0,2.0,1.57;1.0,2.0,3.14`

Example (explicit loop config):

```bash
ros2 launch arena_bringup sim_harness.launch.py \
  enable_autonomy_mode_manager:=true \
  autonomy_goal_mode:=loop_waypoints \
  autonomy_waypoints:="1.0,1.0,0.0;3.0,1.0,0.0;3.0,2.0,1.57;1.0,2.0,3.14" \
  autonomy_loop_on_success:=true \
  autonomy_skip_failed_waypoint:=true
```

## Custom BT XML

Use custom tree with sim harness:

```bash
ros2 launch arena_bringup sim_harness.launch.py \
  enable_autonomy_mode_manager:=true \
  autonomy_goal_behavior_tree:=/home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_bringup/behavior_trees/custom_bt_variant.xml
```

## NT4 integration

### Publish Nav2 velocity/status to NT4

```bash
ros2 launch arena_bringup navigation.launch.py \
  params_file:=/home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
  enable_cmd_vel_nt4_bridge:=true \
  nt4_team:=9214
```

### Read robot mode from NT4 and drive autonomy manager

```bash
ros2 launch arena_bringup navigation.launch.py \
  params_file:=/home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
  enable_nt4_mode_bridge:=true \
  nt4_mode_team:=9214 \
  nt4_mode_table:=ROS \
  nt4_mode_key:=robot_mode \
  ros_mode_topic:=robot_mode \
  enable_autonomy_mode_manager:=true \
  autonomy_mode_topic:=robot_mode \
  autonomy_mode_value:=autonomous
```

## Debug commands

TF chain:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

Topic sanity:

```bash
ros2 topic hz /odom
ros2 topic hz /cmd_vel_nav
ros2 topic echo /map --once
```

Navigator logs:

```bash
grep -n "bt_navigator" ~/.ros/log/latest/launch.log
find ~/.ros/log/latest -maxdepth 1 -type f | grep -E "bt_navigator|navigate_to_pose|nav2"
```

Node details:

```bash
ros2 node info /bt_navigator
ros2 node list
```

## Common issues

- `Failed to create plan with tolerance ...`
  - Start/goal is invalid or too close to map edge. In sim, keep poses away from boundaries.

- `Costmap timed out waiting for update`
  - Check `/map` exists and TF chain (`map -> odom -> base_link`) is present.

- `KeepoutFilter/SpeedFilter mask was not received`
  - Disable zones for sim (`use_keepout_zones:=false use_speed_zones:=false`) unless filter mask servers are intentionally running.

- `Failed to make progress`
  - Ensure robot pose actually changes when controller publishes commands (sim harness now uses `follow_cmd_vel` by default).

## Test scripts

1. `team9214_ws/src/arena_bringup/arena_bringup/tag_test_monitor.py`

- Prints distance/bearing/depth for every currently detected AprilTag
- Prints estimated arena pose `(x,y,yaw)` from TF `map -> base_link`
- Falls bck to `/tag_global_pose` if TF pose is unavailable

Running:

`ros2 run arena_bringup tag_test_monitor`

Running with flags:

```bash
ros2 run arena_bringup tag_test_monitor --ros-args \
  -p detections_topic:=/camera_1/tag_detections \
  -p camera_frame:=camera_1_optical_frame \
  -p map_frame:=map \
  -p base_frame:=base_link \
  -p global_pose_topic:=/tag_global_pose
```