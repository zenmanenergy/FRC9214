# localization_fusion

## Summary

Using common `robot_localization` setup of two EKF, each with a different purpose:

1. local EKF (`odom` filter)
    - Objective: stable short-term motion, no global jumps
    - publishes `odom -> base_link` using smooth local sensors (wheel odom + IMU)

2. Global EKF (`map` filter)
    - Objective: global drift correction
    - pubishes `map -> odom` using global corrections (AprilTag, GPS, AMCL, etc) plus local inputs

Result: `map -> odom -> base_link`

## Files

### `ekf_map.yaml`
- Is the **map filter style** config (`world_frame: map`)
- Expects `odom -> base_link` to already exist from another source/filter.  If not it **BREAKS** the TF chain

### `ekf_odom.yaml`
- Is the **odom filter style** config (`odom)`
- robot_localization local filter (`odom -> base_link`)

### `bringup_single_cam.launch.py`
- Starts only one EKF (`ekf_map`), so if no separate odom publishes/filter exists you'll see missing TF link

## Debugging

1. Check existing TF links:
- `ros2 run tf2_ros tf2_echo map odom`
- `ros2 run tf2_ros tf2_echo odom base_link`
- `ros2 run tf2_ros tf2_echo map base_link`

2. Check EKF inpputs exist:
- `/wheel/odom`
- `/imu/data`
- `/tag_global_pose`

3. Check node list for one vs two EKF's
- `ros2 node list | rg ekf`

### Results
- Expected results in proper dual-filter setup
    - `odom -> base_link` always present (from local EKF).
    - `map -> odom` present when global filter runs.
    - `map -> base_link` resolves through both.