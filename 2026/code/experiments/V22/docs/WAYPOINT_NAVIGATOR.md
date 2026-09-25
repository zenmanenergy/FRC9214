# Waypoint Navigator System

## Overview
The Waypoint Navigator is a stage-based autonomous navigation system that guides the robot through a series of waypoints. It plans a path between waypoints (either as discrete points or smooth splines) and autonomously executes the movement using the swerve drive.

## Architecture

### Core Navigation Loop
The navigator operates as a finite state machine with 4 stages per waypoint:
1. **Stage 0: Initialize** - Get ready for next waypoint
2. **Stage 1: Rotate** - Turn to face the target waypoint
3. **Stage 2: Move** - Drive forward to reach the target position
4. **Stage 3: Complete** - Transition to the next waypoint

### Key Components

**WaypointNavigator** (`waypoint_navigator.py`)
- Main controller for autonomous path following
- Manages waypoint list, current index, and stage progression
- Handles velocity profiling and PID-based movement
- Supports both discrete waypoints and smooth spline following

**CatmullRomSpline** (`catmull_rom.py`)
- Smooth continuous curve interpolation through waypoints
- Reduces jerky movements and creates natural paths
- Calculates look-ahead points for smooth heading tracking
- Total distance computation for progress monitoring

## Navigation Modes

### Discrete Waypoint Mode
- Robot visits each waypoint as a discrete target
- Simple, predictable behavior
- Each waypoint has its own heading target
- Ideal for game positions or precise stopping points

### Spline Following Mode
- Smooth interpolation creates a continuous curve through waypoints
- Robot follows the curve smoothly without sharp turns
- Heading calculated from curve tangent (always facing forward)
- Better for long autonomous routines or field traversal
- Reduces mechanical stress and improves speed consistency

## Stage Details

### Stage 1: Rotation
- Calculate desired heading to face next waypoint
- Use PID controller (`pid_rotate`) to reach target angle
- **Rotation Tolerance**: 5° (configurable)
- **Max Rotation Speed**: 0.8 power (configurable)
- **Timeout**: 10 seconds per stage
- Once rotation error < tolerance, advance to Stage 2

### Stage 2: Movement
- Drive forward using PID controller (`pid_drive`)
- Maintain target heading from Stage 1
- **Velocity Profiling**: Smooth acceleration and deceleration
  - Acceleration: +0.03 power per loop (~50Hz)
  - Deceleration: +0.05 power per loop (harder braking)
  - Starts braking 150cm before target
- **Position Tolerance**: 25cm (configurable)
- **Min Drive Speed**: 0.15 power (above joystick deadzone)
- Once position error < tolerance, advance to Stage 3

### Stage 3: Complete
- Waypoint reached
- Optionally rotate to final waypoint heading
- Transition to next waypoint or stop if none remain

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
  "navigation_use_spline": true,
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

### Spline Control
- `use_spline=true` - Follow smooth interpolated path
- `use_spline=false` - Visit discrete waypoints

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
