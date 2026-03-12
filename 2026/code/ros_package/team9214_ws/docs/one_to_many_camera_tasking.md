# Cameras

## Minimal “single-camera first” bring-up (then scale to 4)

This is the fastest path to a working localization loop.

### Step 1 — Bring up TF + camera stream

**Goal:** you can see `base_link` and `camera_1_optical_frame` in `TF`, and `images` are publishing.

- Launch arena_description/launch/description.launch.py:

    `runs robot_state_publisher with your URDF`

    optionally runs `joint_state_publisher_gui` (not required for fixed joints)

- Then launch your camera driver so you have:

    `/camera_1/image_raw`

    `/camera_1/camera_info` (calibrated intrinsics)

- Acceptance

    `ros2 run tf2_tools view_frames` shows `base_link` -> `camera_1_optical_frame`

    `rqt_image_view` shows camera image

### Step 2 — AprilTag detection for camera_1

**Goal:** output detections with correct IDs and stable tag poses.

- Launch detector node configured with:
    - correct tag family
    - correct tag size
    - correct `camera_frame` (`camera_1_optical_frame`)
    - correct camera topics

- Acceptance
    - you see `/camera_1/tag_detections`
    - `RViz` markers (or printed output) show tag IDs that match reality
    - if you move the robot, tag pose in camera frame changes sensibly

### Step 3 — Convert detections to a map-frame base pose observation

**Goal:** publish `/tag_global_pose` in map.
- Run `tag_pose_observer` with:
    - `tag_map_yaml = arena_tags.yaml`
    - subscribe to `/camera_1/tag_detections`
    - read TF `base_link` <-> `camera_1_optical_frame`
    - output `/tag_global_pose` (PoseWithCovarianceStamped in map)

- Acceptance
    - With robot stationary, `/tag_global_pose` is stable (small noise)
    - If robot moves, pose follows correctly (no axis flips)
    - If tag IDs swap or you face different walls, it still returns a plausible pose

### Step 4 — Add odometry/IMU fusion (so pose is continuous when tags drop out)

**Goal:** publish TF `map->odom` and maintain stable localization through occlusions.

- Run robot_localization:
    - “odom filter” (optional but common): wheel + IMU → `odom`->`base_link`
    - “map filter”: wheel/IMU + `/tag_global_pose` → `map->odom`

Acceptance
- `map->odom` exists and updates when tag observations come in
- When tags are absent, pose continues smoothly using odom (drifts slowly)

### Step 5 — Scale to 2–4 cameras
- Do this incrementally.
    1. Add camera_2 frames to URDF and ensure TF is correct
    2. Add camera_2 intrinsics + image topics
    3. Add detector for camera_2 → /camera_2/tag_detections
    4. Extend observer to consume both topics
    5. Add gating:
        - per-detection decision margin threshold
        - reject implausible jumps vs last accepted pose
        - if two cameras disagree, choose the one with better quality / lower reprojection error proxy

- Acceptance
    - With any one camera covered, localization still works from others
    - Pose doesn’t “flip” when switching which camera sees the tag

## Minimal Launch

Minimal launch wiring (what bringup should do)

Your `arena_bringup/launch/bringup_single_cam.launch.py` should start, in this order:
1. robot_state_publisher (TF)
2. camera driver (publishes image + camera_info)
3. apriltag detector (subscribes camera)
4. tag_pose_observer (subscribes detections + loads tag map)
5. robot_localization (subscribes odom/imu + tag_global_pose)
6. RViz (optional)

## Running

### Single camera

```bash
ros2 launch arena_bringup bringup_single_cam.launch.py \
  camera_ns:=camera_1 \
  tag_map_yaml:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/arena_tag_map/config/757-lab_arena_tags.yaml
```

### Multiple cameras:

```bash
ros2 launch arena_bringup bringup_multi_cam.launch.py \
  camera_names:=camera_1,camera_2,camera_3 \
  tac_map_yaml:=tag_map_yaml:=/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/arena_tag_map/config/757-lab_arena_tags.yaml
```