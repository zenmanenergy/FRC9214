# Tasking
- Arena coordinate system + tag map
- Build robot TF tree (URDF/Xarco)
- Camera calibration
- April detection (bring-up + verification)
- Detections to  global pose (camera → base → map)
- Fuse camera observations over time
- RViz validation and acceptance tests
- Package (structure, launch, params, configs)
- Behavior Tree Layer
- Reliability

## Summary of tasking
1. Arena tag map + URDF frame tree (no vision yet)
2. One camera intrinsics + detection + RViz tag markers
3. Camera extrinsics + convert detection → global pose observation
4. Add wheel/IMU odom + robot_localization fusion
5. Add second/third camera + arbitration/weighting
6. Bag-based regression tests
7. Nav2 + BT autonomy

## Summary of Key Config files
- Map conventions, arena coordinate frame, tag layout
    `arena_tag_map/config/<name>_arena_tags.yaml`
- TF 
    `arena_description/urdf/robot.urdf.xacro (multi-camera frames)`
- Detections to Global Pose
    `tag_pose_observer/config/observer.yaml`
- `robot_localization` “global” filter
    `localization_fusion/config/ekf_map.yaml`


## Package Structure

See `team9214/docs/package_details.md`

## Define the Transforms and end-state interfaces

### Deliverables - TF tree and message contracts

1. Frames you will publish and consume:
    - `map` (arena/world)
    - `odom` (continuous, drift-allowed)
    - `base_link` (robot body)
    - `camera_i_optical_frame` (per camera)

2. Core topics you want:
    - Robot pose estimate (e.g., `/amcl_pose`, `/robot_pose`, `/tf`)
    - usb_cam node topics:
        - namespace: `camera`
            - `/image_raw`
            - `/camera_info`
    - rectify_node
        - namespace: `camera`
            - `/image_rect`
    - apriltag_node
        - namespace: `camera`
            - `/image_rect`
            - `/detections`

    - Tag detections (per camera), and optional “fused detections”

3. Update rate targets (e.g., pose @ 30–50 Hz if using odom fusion; global updates at 5–20 Hz)
    - Camera framerate: 30 Hz

**Why First** Everything else (URDF, calibration, fusion) is easier when you’ve committed to a TF tree and message contracts.

## Establish arena coordinate system + tag map (ground truth)

### Deliverables

1. A single *arena/world* frame definition (call it map):
    - Origin placement (corner vs **center**)
    - Axes convention (x forward, y left, z up is typical in ROS)
    - Units (meters)

2. `AprilTag` layout file (source of truth):
    - Tag ID → pose in map (x,y,z + quaternion)
    - Tag size(s)
    - Tag family (e.g., `tag36h11`)

**Notes**
- Keep this layout in a versioned `YAML` (or `JSON`) so it’s easy to update and reuse across sim/hardware.
- If the arena is planar and tags are on walls, encode correct wall normals; wrong orientation is the #1 silent failure.

## Build the robot *TF tree* (*URDF*/*Xacro*) before calibration

### Deliverables

1. URDF/Xacro defining:
    - `base_link`
    - For each camera: a rigid transform `base_link` -> `camera_i_link` -> `camera_i_optical_frame`

2. A `robot_state_publisher` launch that publishes those static transforms.
    `/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/narwhal_robot/launch/robot_description.launch.py`

    `/home/robots/source_code/first_robotics_comp/frc/team9214_ws/src/narwhal_robot/launch/static_cameras_tf.launch.py`

### Notes
- Even if your camera extrinsics aren’t final yet, you need the structure in place.
- Use `camera_optical_frame` convention (REP-103/REP-105 style: optical frame has **+Z forward**, **+X right**, **+Y down**).

## Camera calibration pipeline (intrinsics first, then extrinsics)

### Intrinsics per camera (mandatory)

#### Deliverables

1. camera_info YAML per camera:
    - focal lengths, principal point, distortion model, distortion coefficients

2. A repeatable calibration procedure documented in your repo

#### Notes
- Do intrinsics on the exact resolution/FPS you’ll use for detection.

### Time sync + transport sanity (mandatory for multi-cam)

#### Deliverables
1. Confirm timestamps are valid and consistent across cameras:
    - hardware timestamps or a single host clock

2. Decide transport:
    - raw sensor_msgs/Image vs image_transport compressed

**Why here:** bad timestamps destroy multi-camera fusion and make pose “jump.”

### Extrinsics (camera-to-base) (strongly recommended)

#### Deliverables

1. `base_link` -> `camera_i_link` transform values in URDF (or a calibration YAML loaded into `static_transform_publisher`)

2. Validation procedure:
    - place robot at known pose, verify projected tag geometry matches

#### Notes
- Extrinsics can be measured (CAD/tape) initially, then refined via calibration (hand-eye / optimization) later.


## AprilTag detection per camera (bring-up + verification)

### Deliverables

1. One detection node per camera (or one node handling N cameras) producing:
    - detections with pose of tag relative to camera optical frame
    - covariance/quality metrics if available

2. A “detection debug view”:
    - annotated image showing tag corners/IDs
    - `rqt_image_view` + `rviz` markers

### Acceptance checks
- Correct tag IDs
- Reasonable range and orientation
- No mirrored axes (optical frame mismatch shows up here)

## Convert detections into global pose observations (camera → base → map)

At this point you have:
    - `T_map_tag` from your tag layout
    - `T_cam_tag` from detector
    - `T_base_cam` from URDF calibration

You can compute an observation of robot pose:
    - `T_map_base` = `T_map_tag` * inverse(`T_cam_tag`) * inverse(`T_base_cam`)
(or equivalent chain depending on which direction your detector outputs)

### Deliverables
1. A “pose observation” node that:
    - subscribes to detections + tag map
    - outputs per-detection geometry_msgs/PoseWithCovarianceStamped in map
    - rejects outliers (bad tags / low margin / steep angle / huge reprojection error)

### Notes
- This node is the seam between “vision” and “localization.” Keep it clean and testable.

## Fuse observations across cameras and over time
You have two common architectures:

### Option A: “Vision provides global pose updates” + fuse with wheel/IMU in robot_localization

#### Deliverables

1. ekf_localization_node for odom -> base_link using wheel odom + IMU

2. Another ekf_localization_node (or same one configured carefully) that consumes:
    - global pose updates from tags (map frame)

3. Publish TF:
    - map -> odom from the filter
    - odom -> base_link from odom fusion

Why good: robust, standard ROS pattern, handles dropout.

### Option B: Full pose graph / factor graph (more complex)

Use something like GTSAM-style factors (tags as landmarks). Better long-term, but heavier.

#### Minimum deliverables for fusion (either option)
1. Multi-camera arbitration:
    - choose best camera per time window OR average multiple observations with covariance weighting

2. Temporal filtering:
    - reject single-frame spikes
    - require consistency across N frames before “hard” resets

## RViz validation and quantitative acceptance tests

### Deliverables

1. RViz config:
    - show map, odom, base_link, camera frames
    - show detected tag poses and expected tag poses
    - show robot pose trace

2. “Known pose” tests:
    - put robot at 3–5 measured spots in arena, record error

3. Bagging:
    - record rosbag2 of images/detections/TF for regression tests

Why now: you need proof your coordinate math + TF is correct before you build autonomy behaviors.

## Package structure, launch system, parameters, and configs

### Deliverables
1. A ROS 2 package layout that cleanly separates:
    - description/ (URDF/Xacro)
    - config/ (camera intrinsics, extrinsics, tag map, filter params)
    - launch/ (bring-up: cameras, detection, localization, viz)

2. Parameter files for:
    - detector thresholds
    - tag sizes/families
    - filter tuning (robot_localization)

## Nav2 + Behavior Tree layer (only after localization is stable)

### Deliverables

1. Nav2 bring-up configured to use your localization TF (map->odom) and robot footprint

2. Costmaps aligned to arena/map

3. BTs that assume localization exists (examples):
    - “Go To Pose”
    - “Follow Path”
    - Recovery behaviors (spin, backup) that don’t corrupt localization

**Key point**: BTs are “downstream.” If localization isn’t solid, BT tuning is wasted.

## Hardening for competition/field use

### Deliverables

1. Startup sequencing:
    - camera_info ready before detector starts
    - static TF published before detections processed

2. Failure modes:
    - what happens when no tags visible
    - what happens when one camera dies

3. Performance:
    - CPU/GPU budgets, per-camera FPS caps

4. Diagnostics:
    - /diagnostics status for each camera and detector
    - latency metrics

---

## A sensible “do it in order” milestone plan
1. Arena tag map + URDF frame tree (no vision yet)
    - AprilTag base localization : `arena_tag_map/config/arena_tags.yaml`
2. One camera intrinsics + detection + RViz tag markers
3. Camera extrinsics + convert detection → global pose observation
4. Add wheel/IMU odom + robot_localization fusion
5. Add second/third camera + arbitration/weighting
6. Bag-based regression tests
7. Nav2 + BT autonomy

---

## Minimal “single-camera first” bring-up (then scale to 4)

This is the fastest path to a working localization loop.

### Step 1 — Bring up TF + camera stream

**Goal**: you can see `base_link` and `camera_1_optical_frame` in TF, and images are publishing.

- Launch `arena_description/launch/description.launch.py`:
    - runs `robot_state_publisher` with your URDF
    - optionally runs `joint_state_publisher_gui` (not required for fixed joints)

- Then launch your camera driver so you have:
    - `/camera_1/image_raw`
    - `/camera_1/camera_info` (calibrated intrinsics)

- Acceptance
    - `ros2 run tf2_tools view_frames` shows `base_link -> camera_1_optical_frame`
    - `rqt_image_view` shows camera image

### Step 2 — AprilTag detection for camera_1

**Goal**: output detections with correct IDs and stable tag poses.

- Launch detector node configured with:
    - correct tag family
    - correct tag size
    - correct `camera_frame` (`camera_1_optical_frame`)
    - correct camera topics

- Acceptance
    - you see `/camera_1/tag_detections`
    - RViz markers (or printed output) show tag IDs that match reality
    - if you move the robot, tag pose in camera frame changes sensibly

### Step 3 — Convert detections to a map-frame base pose observation

**Goal**: publish `/tag_global_pose` in map.

- Run `tag_pose_observer` with:
    - `tag_map_yaml` = `arena_tags.yaml`
    - subscribe to `/camera_1/tag_detections`
    - read TF `base_link` <-> `camera_1_optical_frame`
    - output `/tag_global_pose` (PoseWithCovarianceStamped in map)

- Acceptance
    - With robot stationary, `/tag_global_pose` is stable (small noise)
    - If robot moves, pose follows correctly (no axis flips)
    - If tag IDs swap or you face different walls, it still returns a plausible pose

### Step 4 — Add odometry/IMU fusion (so pose is continuous when tags drop out)

**Goal**: publish TF map->odom and maintain stable localization through occlusions.

- Run robot_localization:
    - “odom filter” (optional but common): wheel + IMU → `odom->base_link`
    - “map filter”: wheel/IMU + `/tag_global_pose` → `map->odom`

- Acceptance
    - map->odom exists and updates when tag observations come in
    - When tags are absent, pose continues smoothly using odom (drifts slowly)

### Step 5 — Scale to 2–4 cameras

**Do this incrementally.**

1. Add camera_2 frames to URDF and ensure TF is correct
2. Add camera_2 intrinsics + image topics
3. Add detector for camera_2 → `/camera_2/tag_detections`
4. Extend observer to consume both topics
5. Add gating:
    - per-detection decision margin threshold
    - reject implausible jumps vs last accepted pose
    - if two cameras disagree, choose the one with better quality / lower reprojection error proxy

- Acceptance
    - With any one camera covered, localization still works from others
    - Pose doesn’t “flip” when switching which camera sees the tag

### Minimal launch wiring (what bringup should do)

Your `arena_bringup/launch/bringup_single_cam.launch.py` should start, in this order:

1. `robot_state_publisher` (TF)
2. camera driver (publishes image + `camera_info`)
3. apriltag detector (subscribes camera)
4. `tag_pose_observer` (subscribes detections + loads tag map)
5. robot_localization (subscribes odom/imu + `tag_global_pose`)
6. RViz (optional)

## Running *-camera_bring-up

### Single

```bash
ros2 launch arena_bringup bringup_single_cam.launch.py \
  camera_ns:=camera_1 \
  tag_map_yaml:=/path/to/arena_tags.yaml
```

### Multi-camera

```bash
ros2 launch arena_bringup bringup_multi_cam.launch.py \
  camera_names:=camera_1,camera_2,camera_3 \
  tag_map_yaml:=/path/to/arena_tags.yaml
```

