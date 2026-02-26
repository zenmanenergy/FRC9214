# arena_bt_nodes

Starter package for custom BehaviorTree.CPP nodes used by Nav2, plus a helper executable to generate a Groot2 node model XML.

## Build

```bash
cd /home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws
colcon build --packages-select arena_bt_nodes
source install/setup.bash
```

## Generate Groot2 node model XML

```bash
ros2 run arena_bt_nodes generate_bt_model_xml \
  /home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_bringup/behavior_trees/arena_bt_nodes_model.xml
```

## Enable plugin in Nav2

In `arena_bringup/params/nav2_params.yaml` under `bt_navigator.ros__parameters`:

```yaml
plugin_lib_names:
  - arena_bt_nodes
```

## Starter Package

### Build + package metadata
- CMakeLists.txt
- package.xml

### Custom node + registration
- `is_battery_ok_node.hpp`
- `is_battery_ok_node.cpp`
- `register_nodes.hpp`
- `register_nodes.cpp`

### Plugin entrypoint for Nav2 BT loader
- `plugin_entrypoint.cpp`

### Groot2 model generator executable:
- `generate_bt_model_xml.cpp`

### Starter docs/example BT
- `README.md`
- `custom_node_example_bt.xml`

### Generated from this example
- Groot2 node model XML:
  - `arena_bt_nodes_model.xml`

### Validation run
- `colcon build --packages-select arena_bt_nodes`
- `ros2 run arena_bt_nodes generate_bt_model_xml...`

### How to use now
1. Build + source
  ```bash
  cd /home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select arena_bt_nodes
  source install/setup.bash
  ```

2. Regenerate model XML anytime
  ```bash
  ros2 run arena_bt_nodes generate_bt_model_xml \
  /home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws/src/arena_bringup/behavior_trees/arena_bt_nodes_model.xml
  ```

3. Enable plugin in Nav2 (`plugin_lib_names`) in `nav2_params.xml`
  ```xml
  plugin_lib_names:
  - arena_bt_nodes
  ```

### IsBatteryOk example

#### enable `arena_bt_nodes` in Nav2 `plugin_lib_names`
- Enabled custom BT plugin loading in `nav2_params.yaml`
  - `plugin_lib_names`
  - `- arena_bt_nodes`

#### insert `IsBatteryOk` ahead of planning with fixed inputs so behavior remains stable
- Add starter custom node usage in `custom_bt.xml`
  - `<IsBatteryOk min_voltage="11.0" battery_voltage="12.0"/>

#### Run

```bash
cd /home/robots/source_code/github/zenmanenergy/FRC9214/2026/code/ros_package/team9214_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arena_bt_nodes
source install/setup.bash
```

#### Launch usual Nav2 bringup and open Groot2 with:
- BT file: `custom_bt.xml`
- Model file: `arena_bt_nodes_model.xml`