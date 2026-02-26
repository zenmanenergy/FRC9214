## Package Directory

```bash
tag_pose_observer/
  CMakeLists.txt
  package.xml
  resource/
    tag_pose_observer
  tag_pose_observer/
    __init__.py
    observer_node.py
  launch/
    observer.launch.py
  config/
    observer.yaml
  setup.py
```

## Build

```bash
colcon build --packages-select tag_pose_observer
source install/setup.bash
ros2 run tag_pose_observer tag_pose_observer
```

## Verify

```bash
ros2 pkg list | grep tag_pose_observer
```