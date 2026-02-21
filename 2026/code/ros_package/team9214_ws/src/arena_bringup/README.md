# arena_bringup

Provides single location to run all necessary ROS2 nodes

---

## Python Scripts
- `autonomy_mode_manager.py`
  - Starts/cancels Nav2 autonomous behavior based on robot mode topic by sending
    `NavigateToPose` (`send_goal_async`)
  - Explicitly set goal with `goal.msg.behavior_tree` (line 96)


- `cmd_vel_nt4_bridge.py`
- `nt4_mode_bridge.py`
- `sim_inputs.py`

## Launch files

### Simulation harness
- `sim_harness.launch.py`

### Single and multi-camera launch files
- `bringup_single_cam.launch.py`
Launch pipeline for camera -> image_rectifier -> AprilTag Detection -> to EKG node.

- `bringup_multi_cam.launch.py`

### localization launch 
- `localization.launch.py`

### navigation launch

Launches the `Nav2` nodes used for control, planning and behaviors.
- `navigation.launch.py`
  - `bt_navigator` is launched from this file.  This is the BT server.
  - `autonomy_goal_behavior_tree` defined in this file
    - Passed to `autonomy_mode_manager`


Use the behavior tree for high-level autonomy (goals, recoveries, sequencing), not direct motor commands.

In Navigation setup:

- `bt_navigator` drives actions
- `controller_server` / `behavior_server` generate velocity commands, remapped to `cmd_vel_nav` (`navigation.launch.py` (line 171), `navigation.launch.py` (line 214))
- `velocity_smoother` + `collision_monitor` process that and output final `cmd_vel` (`nav2_params.yaml` (lines 506-507))
- `cmd_vel` passed to `pycore` node to get to RoboRIO

robotpy will consume `cmd_vel` with network tables and turns it into wheel/actuator commands (for example a diff_drive_controller, motor driver node, or hardware bridge).

### sim launch

Very simple published messages to enable testing of behavior trees without having to be in the lab or have the robot.

---

## Configuration files

Configuration settings can be done in config/bringup.yaml or passed as runtime flags.

## Params

- `nav2_params.yaml`
  - Configures `bt_navigator`
    - If `goal_behavior_tree` is empty, `Nav2` uses its default `NavigateToPose` BT XML

### Single Camera Example:

```bash
ros2 launch arena_bringup bringup_single_cam.launch.py \
  camera_ns:=camera_1 \
  tag_map_yaml:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/arena_tag_map/config/757-lab_arena_tags.yaml \
```

`config/bringup.yaml` or defaults will be used for parameters except for overriden values provided for `camera_ns` and `tag_map_yaml` parameters.

---

## Building ROS package 

### Full team9214 package

```bash
cd team9214_ws
colcon build --symlink-install arena_bringup
source install/setup.bash
```

### arena_bringup only

```bash
cd team9214_ws
colcon build --packages-up-to arena_bringup
source install/setup.bash
```

---

## Running arena_bringup

### Multi-cam

```bash
ros2 launch arena_bringup bringup_multi_cam.launch.py \
  camera_names:=camera_1,camera_2,camera_3 \
  tag_map_yaml:=/path/to/arena_tags.yaml
```
---

## Run full stack

### 1. Localization + AprilTag pipeline (/tag_global_pose -> EKF)

```bash
cd ~/source_code/first_robotics_comp/frc/team9214_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch arena_bringup bringup_single_cam.launch.py \
  camera_ns:=camera_1 \
  tag_map_yaml:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/arena_tag_map/config/757-lab_arena_tags.yaml \
  observer_yaml:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/tag_pose_observer/config/observer.yaml \
  ekf_map_yaml:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/localization_fusion/config/ekf_map.yaml \
  urdf_xacro:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/arena_description/urdf/robot.urdf.xacro
```

**NOTE** if you want `multi-cam use bringup_multi_cam.launch.py`

### 2. Nav2 stack (includes bt_navigator)

```bash
cd ~/source_code/first_robotics_comp/frc/team9214_ws
source install/setup.bash

ros2 launch arena_bringup navigation.launch.py \
  params_file:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
  autostart:=true
```

### 3. Send a behavior-tree-driven nav goal (optional, 3rd terminal)

```bash
cd ~/source_code/first_robotics_comp/frc/team9214_ws
source install/setup.bash

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## Run full stack with sim_harness

### 0. Setup, provision and source python venv

[TODO] find and add link to the README that has this.

### 1. Build and source

```bash
cd /home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws
colcon build --symlink-install --packages-up-to arena_bringup
source install/setup.bash
```

### 2. Launch full stack + sim harness (Nav2 + bt_navigator + sim imputs + autonomy trigger)

**Note** There is a `launch_navigation` flag that will launch the navigation stack with the sim_harness when set to `true`

```bash
ros2 launch arena_bringup sim_harness.launch.py \
  launch_navigation:=true \
  nav_params_file:=/home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
  nav_autostart:=true \
  enable_autonomy_mode_manager:=true \
  mode_schedule:=disabled:3,teleop:3,autonomous:12,disabled:3 \
  sim_vx:=0.1 \
  sim_wz:=0.0 \
  tag_pose_x:=0.0 \
  tag_pose_y:=0.0 \
  tag_pose_yaw:=0.0
```

### 3. Watch BT/nav activity

In 2nd terminal:

```bash
cd /home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws
source install/setup.bash
ros2 topic echo /navigate_to_pose/_action/status
```

### 4. Confirm mode changes from sim schedule

In 3rd terminal:

```bash
cd /home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws
source install/setup.bash
ros2 topic echo /robot_mode
```

### 5. Optional: Inspect BT navigator code

In 4th terminal:

`ros2 node info /bt_navigator`

### 6. Force autonomous mode (manual override test)

`ros2 topic pub -1 /robot_mode std_msgs/msg/String "{data: autonomous}"`

## Run custom behavior tree for prototyping

### Nav2 behavior tree for NavigateToPose testing.

#### Example 1
- ComputePathToPose -> FollowPath with a simple recovery that clears costmaps and retries once.

  ```bash
  ros2 launch arena_bringup sim_harness.launch.py \
    launch_navigation:=true \
    nav_params_file:=/home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
    enable_autonomy_mode_manager:=true \
    autonomy_goal_behavior_tree:=/home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws/src/arena_bringup/behavior_trees/custom_bt.xml
  ```

#### Example 2: Example 1 + updates
- RateController around ComputePathToPose (replan at 1 Hz)
- PipelineSequence for plan/follow flow
- RoundRobin recovery set:
- clear local/global costmaps
- Spin
- Wait 

  ```bash
  ros2 launch arena_bringup sim_harness.launch.py \
    launch_navigation:=true \
    enable_autonomy_mode_manager:=true \
    autonomy_goal_behavior_tree:=/home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws/src/arena_bringup/behavior_trees/custom_bt_variant.xml
  ```

---

## Verification

### Transforms

#### TF chain

`ros2 run tf2_ros tf2_echo map base_link`

#### Visually 

You can use `rviz2` to visualize transforms.  There may be some setup needed.

### Camera and detections

```bash
ros2 topic list | rg "camera_1|tag_detections|tag_global_pose"
```

## ROS to RoboRIO Interfaces

### NT4 cmd_vel bridge (RoboRIO)

To publish Nav2 velocity commands to NetworkTables (NT4), enable the bridge in navigation launch:

```bash
ros2 launch arena_bringup navigation.launch.py \
  params_file:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
  enable_cmd_vel_nt4_bridge:=true \
  nt4_team:=9214
```

Defaults:
- input topic: `cmd_vel_nav`
- table: `ROS`
- keys: `cmd_vel_x`, `cmd_vel_y`, `cmd_vel_theta`, `cmd_vel` (array), `cmd_vel_stamp`
- status keys:
  - `status_mode` (`String`)
  - `status_cmd_source` (`String`)
  - `status_cmd_valid` (`Boolean`)
  - `status_cmd_age_sec` (`Double`)
  - `status_localization_ok` (`Boolean`)
  - `status_localization_age_sec` (`Double`)
  - `status_nav_state` (`String`)
  - `status_nav_goal_active` (`Boolean`)
  - `status_nav_status_code` (`Integer`)
  - `status_safety_stop` (`Boolean`)
  - `status_safety_state` (`String`)
  - `status_odom_vx`, `status_odom_vy`, `status_odom_wz` (`Double`)
  - `status_heartbeat` (`Double counter`)
  - `status_stamp` (`Double epoch seconds`)

Useful bridge parameters:

| parameter name | optional | default |
| --- | --- | --- |
| `mode_topic` | optional `std_msgs/String` | empty means use `default_mode` |
| `default_mode` | No | default: `unknown` |
| `cmd_source` | No | default: `nav2` |
| `cmd_timeout_sec` | No | default: `0.5` |
| `localization_topic` | No | default: `/tag_global_pose` |
| `localization_timeout_sec` | No | default: `1.0` |
| `nav_status_topic` | No | default: `/navigate_to_pose/_action/status` |
| `odom_topic` | No | default: `/odom` |
| `safety_stop_topic` | optional `std_msgs/Bool` | |
| `safety_state_topic` | optional `std_msgs/String` | |
| `heartbeat_hz` | No | default: `10.0` |

### Mode-triggered autonomy manager

`autonomy_mode_manager` listens for a mode string and triggers Nav2 when the mode
transitions into autonomous.

Default behavior:
- subscribes to `robot_mode` (`std_msgs/String`)
- when value becomes `autonomous`, sends one `NavigateToPose` goal
- when mode exits autonomous, cancels active goal (configurable)

Launch with navigation:

```bash
ros2 launch arena_bringup navigation.launch.py \
  params_file:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
  enable_autonomy_mode_manager:=true \
  autonomy_mode_topic:=robot_mode \
  autonomy_mode_value:=autonomous \
  autonomy_goal_frame_id:=map \
  autonomy_goal_x:=1.0 \
  autonomy_goal_y:=0.0 \
  autonomy_goal_yaw:=0.0
```

Optional:
- `autonomy_goal_behavior_tree:=/absolute/path/to/custom_bt.xml`
- `autonomy_cancel_on_exit:=true|false`

### NT4 -> ROS mode bridge

`nt4_mode_bridge` reads a mode string from NetworkTables and publishes it to a ROS topic
for `autonomy_mode_manager` to consume.

#### Run Nav2 + NT4 mode bridge + autonomy manager together:

```bash
ros2 launch arena_bringup navigation.launch.py \
  params_file:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
  enable_nt4_mode_bridge:=true \
  nt4_mode_team:=9214 \
  nt4_mode_table:=ROS \
  nt4_mode_key:=robot_mode \
  ros_mode_topic:=robot_mode \
  enable_autonomy_mode_manager:=true \
  autonomy_mode_topic:=robot_mode \
  autonomy_mode_value:=autonomous \
  autonomy_goal_frame_id:=map \
  autonomy_goal_x:=1.0 \
  autonomy_goal_y:=0.0 \
  autonomy_goal_yaw:=0.0
```

Default NT4 mode bridge settings:
- table: `ROS`
- key: `robot_mode` (expected values like `teleop`, `autonomous`, `disabled`)
- ROS publish topic: `robot_mode` (`std_msgs/String`)

## Sim harness (no arena hardware)

`sim_harness.launch.py` publishes simulated:
- `/robot_mode` (scripted mode sequence)
- `/tag_global_pose`
- `/odom` and `/wheel/odom`
- TF: `map -> odom` and `odom -> base_link`

and can optionally include your Nav2 stack.

```bash
ros2 launch arena_bringup sim_harness.launch.py \
  nav_params_file:=/home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
  mode_schedule:=disabled:2,teleop:4,autonomous:10,disabled:2 \
  enable_autonomy_mode_manager:=true
```

Useful overrides:
- `launch_navigation:=false` (run publishers only)
- `sim_vx:=0.2` and `sim_wz:=0.3` (moving odom simulation)
- `tag_pose_x:=1.5` `tag_pose_y:=0.5` `tag_pose_yaw:=0.0`
- `enable_nt4_mode_bridge:=true` and/or `enable_cmd_vel_nt4_bridge:=true`

Notes:
- For full Nav2 behavior execution, your Nav2 params still need a valid map configuration
  (`map_server.yaml_filename` and related costmap topics).
- NT4-enabled options still require Python `pyntcore`/`ntcore` in your runtime environment.

---

### Test custom BT XML

```bash
cd /home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws
colcon build --symlink-install --packages-up-to arena_bringup
source install/setup.bash
```

```bash
# Optional sanity check that your BT file exists
ls -l /ABSOLUTE/PATH/to/custom_bt.xml
```

```bash
ros2 launch arena_bringup sim_harness.launch.py \
  launch_navigation:=true \
  use_keepout_zones:=false \
  use_speed_zones:=false \
  nav_params_file:=/home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws/src/arena_bringup/params/nav2_params.yaml \
  nav_autostart:=true \
  enable_autonomy_mode_manager:=true \
  autonomy_goal_behavior_tree:=/ABSOLUTE/PATH/to/custom_bt.xml \
  mode_schedule:=disabled:3,teleop:3,autonomous:12,disabled:3 \
  sim_vx:=0.1 \
  sim_wz:=0.0 \
  tag_pose_x:=0.0 \
  tag_pose_y:=0.0 \
  tag_pose_yaw:=0.0
```

#### In another terminal

```bash
cd /home/robots/source_code/github/sidenoteemail/frc_team9214/team9214_ws
source install/setup.bash
ros2 topic echo /robot_mode
```

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

```bash
ros2 topic pub -1 /robot_mode std_msgs/msg/String "{data: autonomous}"
```

## Helpful debugging

### Node details (when running)

- Syntax:

`ros2 node info /<node>`

- Example:

`ros2 node info /bt_navigator`

### List all running nodes

```bash
$ ros2 node list
/autonomy_mode_manager
/behavior_server
/bt_navigator
/bt_navigator_navigate_through_poses_rclcpp_node
/bt_navigator_navigate_to_pose_rclcpp_node
/collision_monitor
/controller_server
/global_costmap/global_costmap
/lifecycle_manager_navigation
/local_costmap/local_costmap
/planner_server
/route_server
/sim_inputs
/smoother_server
/transform_listener_impl_5573570f8cb0
/transform_listener_impl_64114f8aa440
/velocity_smoother
/waypoint_follower
```


### Generate test ROS2 data

- Publish `/robot_mode` as `autonomous`

```bash
ros2 topic pub -r 2 /robot_mode std_msgs/msg/String "{data: autonomous}"
```

- Publish test `/tag_global_pose` localization data

```bash
ros2 topic pub -r 20 /tag_global_pose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.05, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1]}}"
```

## Common issues

### No `/tag_global_pose` or `map -> base_link`
- Map
  - Current `map` is the frame name
    - Right now it comes from:
    - `tag_map_yaml` in `bringup.yaml` points to `arena_tags.yaml`
    - `tag_pose_observer` uses those tag poses as ground truth in frame "`map`" and publishes `/tag_global_pose`
    - EKF (`ekf_map.yaml`) uses `map_frame`: map and should publish `map -> base_link`

**IMPORTANT** If you have NO `/camera_1/tag_detections`/`global pose`, `map -> base`_link will stay missing because EKF has no global pose input (and maybe also no `odom`/`imu` input). The map being used is your AprilTag layout file.
- No /camera_1/tag_detections. This is upstream of the map file. If detector publishes nothing, `observer` and `EKF` can’t produce `map -> base_link`.
- No detections = detector/config/camera-view issue.

---

### NO `/camera_1/tag_detections` messages

- You will see `/camera_1/tag_detections` messages with an empty detections: `[]` when no tags are in view.
- If you see NO messages at all, that usually means one of these:
  - `apriltag_node` is not running/crashed
  - topic/remap mismatch (publishing to a different topic)
  - wrong input topic names to apriltag (image / camera_info)
  - incompatible params causing node startup failure or no subscriptions


  ## Build issues

  - Continue to have missing pip packages ddepending on what machine we are building on.  Need to setup requirements.txt to stop these mistakes.  Current issues needed the following to build:
    `ModuleNotFoundError: No module named 'catkin_pkg'`
    `pip install catkin_pkg`
    pip install --upgrade pip
    pip install pyyaml
    pip install jinja2
    pip install typeguard
