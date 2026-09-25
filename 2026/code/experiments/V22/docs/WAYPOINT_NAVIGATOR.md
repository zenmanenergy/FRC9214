# Waypoint Navigator System

## Overview
The Waypoint Navigator is a stage-based autonomous navigation system that guides the robot through a series of waypoints. It drives a straight-line vector to each waypoint in turn and autonomously executes the movement using the swerve drive.

## Architecture

### Core Navigation Loop
The navigator operates as a finite state machine with 2 stages per waypoint:
1. **Stage 1: Drive + Rotate** - Drive a straight line to the target waypoint while gradually blending heading from the robot's actual heading at the start of the leg toward the waypoint's target heading
2. **Stage 2: Dwell** - Pause for the waypoint's configured dwell time, then advance to the next waypoint (or loop/finish)

### Key Components

**WaypointNavigator** (`waypoint_navigator.py`)
- Main controller for autonomous path following
- Manages waypoint list, current index, and stage progression
- Handles velocity profiling and PID-based movement
- Drives a single straight-line leg at a time; no curve fitting

**heading_math** (`swerve/heading_math.py`)
- `shortest_angle_diff(a, b)` / `lerp_angle(a, b, t)` - shared angle-wrap and blend helpers
- Used to gradually blend heading from the leg's start heading to its target heading as translation progresses, so rotation and translation finish together

## Heading Blend
- At the start of each leg, the robot's actual current heading and position are captured (`leg_start_heading`, `leg_start_x/y`) along with the straight-line distance to the target (`leg_total_distance`)
- Every update tick, `progress = 1 - (distance_to_target / leg_total_distance)` and `desired_heading = lerp_angle(leg_start_heading, target_heading, progress)`
- The rotation PID drives toward `desired_heading` instead of snapping directly to the final heading, so the robot always arrives exactly on-heading regardless of where it started the leg

## Stage Details

### Stage 1: Drive + Rotate (combined)
- Calculate desired heading by blending from the leg's start heading toward the target waypoint heading, proportional to translation progress
- Use PID controller (`pid_rotate`) to reach the blended desired heading
- Drive forward using PID controller (`pid_drive`), toward the target position, simultaneously - a single combined movement
- **Velocity Profiling**: Smooth acceleration and deceleration
  - Acceleration: +0.03 power per loop (~50Hz)
  - Deceleration: +0.05 power per loop (harder braking)
  - Starts braking 150cm before target
- **Rotation Tolerance**: 5° (configurable)
- **Position Tolerance**: 25cm (configurable)
- **Max Rotation Speed**: 0.8 power (configurable)
- **Min Drive Speed**: 0.15 power (above joystick deadzone)
- **Timeout**: 10 seconds per stage
- Once position error < tolerance, advance to Stage 2 (dwell)

### Stage 2: Dwell / Advance
- Waypoint reached; pause for the waypoint's configured dwell time
- Transition to next waypoint (capturing a fresh heading blend for the new leg), loop back to the first waypoint, or stop if none remain

## Velocity Profiling

### Smooth Acceleration
```
current_drive_speed += accel_rate * dt
```
- Prevents jerky starts
- Allows smooth ramp-up to max speed

### Intelligent Deceleration
```
if distance_to_target < decel_distance:
    apply increasing brake force
```
- Starts braking 150cm out (configurable)
- Harder deceleration rate (0.05 vs 0.03 accel)
- Prevents overshoot at final waypoint
- Minimum speed floor prevents complete stop until final waypoint

## PID Controllers

### Drive PID (`pid_drive`)
- **Purpose**: Close distance error to waypoint
- **Gains**: kp=0.004, ki=0.0, kd=0.0
- **Output**: Forward motor power
- **Max Integral**: 0.3 (prevent windup)
- **Why no kD?**: Odometry is noisy; derivative amplifies noise

### Rotation PID (`pid_rotate`)
- **Purpose**: Close heading error to target angle
- **Gains**: Loaded from calibration (autotuned or manual)
- **Output**: Rotation motor power
- **Max Integral**: 0.3
- **Typical**: kp~0.01-0.02, ki~0.0, kd~0.0001 (conservative)

## Command Interface (Dashboard)

### Starting Navigation
```json
{
  "navigate_waypoints_command": true,
  "navigation_waypoints_json": "[{\"x\": 0, \"y\": 0, \"heading\": 0}, ...]",
  "navigation_loop": false,
  "navigation_max_speed": 0.8
}
```

### Stopping Navigation
```json
{
  "stop_navigation_command": true
}
```

## Configuration Parameters

### Tuning Tolerances
- **rotation_tolerance**: 5° - how close to target heading required
- **position_tolerance**: 25cm - how close to waypoint required

### Speed Limits
- **max_rotation_speed**: 0.8 power
- **max_move_speed**: 0.8 power
- **min_drive_speed**: 0.15 power (above deadzone)
- **accel_rate**: 0.03 per loop
- **decel_rate**: 0.05 per loop
- **decel_distance**: 150cm from waypoint

### Timeouts
- **timeout_per_stage**: 10 seconds - abort if stage takes too long

## Advanced Features

### Live Tuning
PID gains can be adjusted real-time via SmartDashboard:
- `Nav_kP_drive` - Drive proportional gain
- `Nav_kP_rotate` - Rotation proportional gain
- `Nav_pos_tolerance` - Position tolerance (cm)
- `Nav_rot_tolerance` - Rotation tolerance (degrees)

### Loop Mode
- `loop=true` - Restart at first waypoint after reaching last
- `loop=false` - Stop after final waypoint

## State Management

### Active Navigation
- `is_active=true` - Robot is navigating autonomously
- Pilot controls disabled during navigation
- Dashboard can stop navigation at any time

### Idle State
- `is_active=false` - Waiting for navigation command
- Pilot controls enabled (teleop only)

## Integration with Robot

The robot (`robot.py`) integrates waypoint navigation by:
1. Creating WaypointNavigator with SwerveDrive reference
2. Calling `navigator.update()` every robot cycle
3. Only accepting pilot input when `navigator.is_active=false`
4. Listening for navigation commands from dashboard

## Performance Considerations

- **Odometry Drift**: Accumulates over long paths; mitigated by odometry reset or spline recalibration
- **Motor Limits**: Current limiting engaged if navigation is too aggressive
- **Timeout Protection**: Prevents infinite loops; aborts after 10 seconds per stage
- **Voltage Sensitivity**: Gains interpolated per battery voltage for consistent performance
