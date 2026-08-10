# Swerve Drive Library

A production-ready, type-hinted Python library for FRC swerve drive control with complete sensor fusion, autonomous navigation, and adaptive tuning.

**Version**: 1.0.0  
**Author**: Team 9214

## Features

✅ **Modular Wheel Control** - Independent drive and turn motors per wheel  
✅ **Field-Relative Movement** - Drive using field coordinates, not robot orientation  
✅ **Odometry with IMU Fusion** - Dead reckoning + gyro sensor fusion for accuracy  
✅ **Adaptive PID Tuning** - Auto-calibrated gains correlated to battery voltage  
✅ **Smooth Path Following** - Catmull-Rom splines with arc-length queries  
✅ **Motor Safety** - Current monitoring for collision detection  
✅ **Persistent Calibration** - Encoder offsets and tuning stored in JSON  
✅ **Type Hints** - Full type annotations for IDE support  
✅ **Comprehensive Documentation** - Docstrings with examples on every class  

## Installation

This is a local library. Import it directly:

```python
from swerve import SwerveDrive, SwerveOdometry
```

## Quick Start

### Initialize the System

```python
from swerve import SwerveDrive

swerve = SwerveDrive()
# Loads calibration, initializes wheels, sets up controllers
```

### Teleop Drive (Field-Relative)

```python
# In teleoperated period
forward = joystick.getY()      # -1 to 1
strafe = joystick.getX()       # -1 to 1
rotate = joystick.getTwist()   # -1 to 1

swerve.drive_swerve(forward, strafe, rotate)

# Update position estimate
swerve.odometry.update()

# Fuse gyro into heading estimate
swerve.imu.fuse_heading(swerve.odometry)
```

### Autonomous Straight Drive

```python
# Drive straight at heading 90° with speed 0.5
swerve.drive_straight(speed=0.5, target_angle=90.0)

# Or drive exactly 200 cm, auto-decelerating at endpoint
done = swerve.drive_for_distance(speed=0.5, target_distance_cm=200.0)
```

### In-Place Rotation

```python
# Spin to face 180°
aligned = swerve.drive_to_heading(target_angle=180.0)
if aligned:
    print("Rotation complete!")
```

### Autonomous Path Following

Define waypoints and let the library handle everything:

```python
waypoints = [
    {'x': 0, 'y': 0, 'heading': 0},
    {'x': 100, 'y': 50, 'heading': 45},
    {'x': 200, 'y': 100, 'heading': 90},
]

# Start path following
swerve.follow_path(waypoints, speed=0.6)

# Drive the path (odometry and IMU fusion handled automatically)
while not swerve.is_path_complete():
    swerve.update_autonomous()

# Done!
```

**That's it!** No need to deal with splines, waypoint sampling, or manual state management. Just define waypoints and call two methods.

**Autonomous Methods:**
- `follow_path(waypoints, speed)` - Start following a path
- `update_autonomous()` - Call in loop to drive along path (all updates handled internally)
- `is_path_complete()` - Check if destination reached
- `stop_path()` - Cancel path following

## Core Classes

### SwerveDrive

Main coordinator for the swerve system.

```python
swerve = SwerveDrive()

# Drive methods
swerve.drive_swerve(forward, strafe, rotate)          # Field-relative teleop
swerve.drive_straight(speed, target_angle)             # Straight line
swerve.drive_for_distance(speed, distance_cm)          # Limited distance
swerve.drive_to_heading(target_angle)                  # In-place rotation
swerve.stop_all()                                      # Emergency stop

# Autonomous path following
swerve.follow_path(waypoints, speed=0.5)              # Start following path
swerve.update_autonomous()                             # Update in main loop
swerve.is_path_complete()                              # Check if done
swerve.stop_path()                                     # Cancel path

# State queries
is_moving = swerve.is_moving()                         # Moving or aligning?
is_aligning = swerve.is_aligning()                     # Wheel alignment active?
state = swerve.get_movement_state()                    # 'idle', 'moving', 'aligning'

# Subsystems
swerve.wheels                                          # Dict of SwerveWheel objects
swerve.odometry                                        # SwerveOdometry instance
swerve.imu                                             # SwerveIMU instance
swerve.pid_controllers                                 # Dict of PIDController objects
swerve.tuner                                           # SwerveTuner instance
swerve.calibration                                     # EncoderCalibration instance
```

### SwerveWheel

Individual wheel with drive and turn motors.

```python
wheel = swerve.wheels['front_right']

# Motor control
wheel.set_drive_power(0.5)                             # 50% speed
wheel.set_turn_power(-0.2)                             # Turn slightly
wheel.stop()                                           # Stop both motors

# Angle queries
angle = wheel.get_angle()                              # Adjusted angle 0-360°
raw = wheel.get_raw_angle()                            # Raw encoder 0-360°

# Distance/velocity
distance_cm = wheel.get_drive_distance()               # Total distance traveled
velocity_rpm = wheel.get_drive_velocity()              # Current speed

# Calibration
wheel.set_zero_offset(45.0)                            # Mark current as 45°
offset = wheel.get_zero_offset()                       # Retrieve offset
```

### SwerveOdometry

Dead reckoning position and heading estimate.

```python
odom = swerve.odometry

# Update from wheel encoders
distance_this_cycle = odom.update()                    # Call once per loop

# Pose queries
x, y = odom.get_position()                             # Position in cm
heading = odom.get_heading()                           # Heading 0-360°
distance = odom.get_total_distance()                   # Total distance in cm

# Corrections (call with sensor fusion data)
odom.set_position(100.0, 200.0)                        # Camera/AprilTag correction
odom.set_heading(45.0)                                 # IMU heading fusion
odom.reset()                                           # Zero everything

# Kinematics
heading_delta = odom.get_last_heading_delta()          # Last update's rotation
```

### SwerveIMU

NavX2 gyro wrapper with sensor fusion.

```python
imu = swerve.imu

# Status
is_ready = imu.is_ready()                              # Calibrated and connected?
is_connected = imu.is_connected()                      # Connected?
is_calibrating = imu.is_calibrating()                  # Still calibrating on startup?

# Heading
heading = imu.get_heading()                            # Heading 0-360° CCW+
imu.zero_heading()                                     # Zero at current orientation

# Orientation (for tilt/rollover detection)
pitch = imu.get_pitch()                                # Forward/backward tilt
roll = imu.get_roll()                                  # Left/right lean

# Sensor fusion
imu.fuse_heading(odom)                                 # Blend IMU into odometry heading
                                                       # 95% IMU, 5% wheel kinematics

# Dashboard
imu.publish_dashboard()                                # Post to SmartDashboard
```

### PIDController

Standard PID with integral anti-windup and autotuning.

```python
pid = PIDController(kp=0.003, ki=0.005, kd=0.0001, name='wheel_turn')

# Manual control loop
error = target_angle - current_angle
output = pid.calculate(error)                          # Get control output
motor.set(output)

# Tuning
pid.set_gains(0.004, 0.006, 0.00015)                  # Update gains
pid.reset()                                            # Zero state

# Autotuning (relay method)
result = pid.autotune(
    get_error_func=lambda: target - current,
    set_output_func=motor.set,
    max_power=0.5,
    duration_seconds=15.0,
    target_cycles=3
)
if result['success']:
    print(f"Tuned: kp={result['kp']:.6f}")
```

### EncoderCalibration

Persistent storage of encoder offsets and PID gains.

```python
cal = swerve.calibration

# Wheel offsets
cal.set_offset('front_right', 45.0)                    # Set offset in degrees
offset = cal.get_offset('front_right')                 # Retrieve
cal.save_calibration()                                 # Write to disk

# Tuning history
cal.add_tuning_result(battery_voltage=12.0, wheel_gains={
    'front_left': {'kp': 0.003, 'ki': 0.005, 'kd': 0.0001},
    'front_right': {'kp': 0.003, 'ki': 0.005, 'kd': 0.0001},
    'rear_left': {'kp': 0.003, 'ki': 0.005, 'kd': 0.0001},
    'rear_right': {'kp': 0.003, 'ki': 0.005, 'kd': 0.0001},
})

# Interpolated gains at current battery voltage
gains = cal.get_interpolated_gains(battery_voltage=12.0)
```

### SwerveTuner

Automated per-wheel PID tuning system.

```python
tuner = swerve.tuner

# Start tuning sequence
tuner.start()

# In main robot loop, call update each cycle
while tuner.is_active():
    tuner.update()
    # Tuner measures response at 0°, 90°, 180°, 270°
    # Calculates per-wheel gains and saves to calibration

# View tuning results
tuner.publish_tuning_history()                         # Post to SmartDashboard
```

### CatmullRomSpline

Smooth C1-continuous curves through waypoints with simple distance-based queries.

```python
waypoints = [
    {'x': 0, 'y': 0, 'heading': 0},
    {'x': 100, 'y': 50, 'heading': 45},
    {'x': 200, 'y': 100, 'heading': 90},
]

spline = CatmullRomSpline(waypoints)

# Query any point directly (recommended)
state = spline.get_state_at_distance(50.0)                # At 50cm
print(f"({state['x']:.1f}, {state['y']:.1f}) @ {state['heading']:.1f}°")

# Iterate through path at regular intervals
for state in spline.sample_path(step_cm=10.0):            # Every 10cm
    print(f"Distance {state['distance']:.1f}cm: "
          f"({state['x']:.1f}, {state['y']:.1f}) @ {state['heading']:.1f}°")

# Get total path length
total = spline.get_total_distance()                        # Arc length in cm

# Access individual waypoints
waypoint = spline.get_waypoint(0)                          # First waypoint

# Advanced: Lower-level access for fine control
segment, t, distance = spline.find_segment_for_distance(50.0)
pos = spline.evaluate(segment, t)
heading = spline.interpolate_heading(segment, t)
tangent = spline.evaluate_tangent(segment, t)
```

**Recommended Methods:**
- `get_state_at_distance(distance_cm)` - Query complete state (x, y, heading) at any distance
- `sample_path(step_cm)` - Iterator through path at regular intervals
- `get_waypoint(index)` - Get original waypoint by index
- `get_total_distance()` - Total arc-length of path in cm

**Advanced Methods (lower-level):**
- `find_segment_for_distance()` - Segment lookup for fine control
- `evaluate()` - Position evaluation
- `interpolate_heading()` - Heading calculation
- `evaluate_tangent()` - Tangent vector
- `get_heading_from_tangent()` - Custom heading from tangent

## Configuration

Edit `swerve_config.py` to configure hardware:

### Wheel Configuration

```python
WHEELS = {
    "front_right": {
        "drive_canid": 10,        # SparkMax CAN ID for drive motor
        "turn_canid": 11,         # SparkMax CAN ID for turn motor
        "encoder_dio": 0,         # DIO port for absolute encoder
        "button": 4,              # Gamepad button for calibration (Y)
        "manual_offset": 0.0,     # Physical rotation offset in degrees
        "rotation_angle": 225,    # Angle for 360° in-place spin
        "position": {
            "x": 0.5,             # Relative position (-0.5 to 0.5)
            "y": 0.5,             # Relative position (-0.5 to 0.5)
        },
    },
    # ... front_left, rear_left, rear_right
}
```

### Robot Dimensions

```python
ROBOT_TRACKWIDTH_CM = 56.0    # Left-to-right center-to-center distance
ROBOT_WHEELBASE_CM = 53.0     # Front-to-rear center-to-center distance
```

### Control Parameters

```python
MOTOR_SCALE_ALIGN = 0.3       # 30% max speed for wheel alignment
MOTOR_SCALE_TELEOP = 0.75     # 75% max speed for teleop drive

ALIGN_TOLERANCE = 5.0         # Angle error tolerance for alignment (degrees)
ALIGN_TIMEOUT = 5.0           # Timeout for alignment attempts (seconds)
DRIVE_ANGLE_TOLERANCE = 5.0   # Tolerance for drive straight heading (degrees)
```

### Storage

```python
OFFSET_FILE = "/home/lvuser/encoder_offsets.json"   # RoboRIO file path
```

## Coordinate System

**Field Frame** (used for odometry and autonomous):
- **X-axis**: Right side of field (positive right)
- **Y-axis**: Away from alliance station (positive forward)
- **Heading**: 0° = +X direction, increases counter-clockwise
- **Rotation**: CCW positive (right-hand rule)

**Robot Frame** (used internally):
- **Forward**: Robot's front direction
- **Right**: Robot's right side
- **Wheel angles**: 0°=backward, 90°=right, 180°=forward, 270°=left

## Calibration Workflow

### 1. Encoder Offset Calibration

Point each wheel to a known direction (forward), then:

```python
swerve.set_wheel_zero('front_right')  # Mark current angle as 0°
swerve.calibration.save_calibration()
```

### 2. PID Tuning

Run automated tuning:

```python
swerve.tuner.start()
# Let it run for ~2 minutes while wheels oscillate
# Gains are automatically saved when complete
```

### 3. Battery Voltage Correlation

Repeat tuning at different battery voltages (12V, 11V, 10V, etc.) to enable adaptive gains.

## Motor Configuration

**Current Setup** (from `swerve_wheel.py`):
- Drive motors: SparkMax brushless
- Turn motors: SparkMax brushless
- Idle mode: Coast (default)
- Encoder: DutyCycleEncoder (absolute)

To enable **brake mode**, add to `SwerveWheel.__init__`:

```python
self.drive_motor.setIdleMode(SparkMax.IdleMode.kBrake)
self.turn_motor.setIdleMode(SparkMax.IdleMode.kBrake)
```

## Safety Features

### Motor Current Monitoring

Collision detection via drive motor current:

```python
swerve.update_motor_currents()

# Check for alerts
if swerve.has_current_alert('front_right'):
    print("Front right motor overload!")

# Query current
current = swerve.get_motor_current('front_right')
```

Thresholds:
- **Spike threshold**: 45A (collision)
- **Sustained threshold**: 35A average (mechanical binding)
- **Alert cooldown**: 10 cycles

### Smooth Acceleration

Drive ramping on stop to prevent wheel slip:

```python
# Configured in SwerveDrive.__init__
self.power_ramp_rate = 0.2  # Reduce to zero at 20% per cycle
```

## Performance Specifications

| Parameter | Value |
|-----------|-------|
| Max wheel speed | 15 ft/s (configurable) |
| Max rotation speed | 360°/s (in-place) |
| Alignment accuracy | ±5° |
| Response time (wheel align) | <500ms |
| Odometry update rate | 50Hz (20ms) |
| Motor current monitoring | Real-time |

## Troubleshooting

### Wheels Not Aligning

1. **Check encoder offsets**: Verify encoders read correct angles
   ```python
   print(swerve.wheels['front_right'].get_angle())  # Should match physical angle
   ```

2. **Check PID gains**: Autotuning may need to run again at current battery voltage

3. **Verify motor direction**: Negative signs in `set_drive_power()` may need adjustment

### Odometry Drift

1. **Fuse IMU heading**: Make sure `imu.fuse_heading(odom)` is called every loop
2. **Check wheel diameters**: Verify `WHEEL_DIAMETER_CM` in odometry math
3. **Verify trackwidth/wheelbase**: Measure robot dimensions carefully

### Jerky Movement

1. **Increase alignment tolerance**: Raise `ALIGN_TOLERANCE` slightly
2. **Reduce angle scale factor**: Lower `cos(error)` scaling in `drive_swerve()`
3. **Enable brake mode**: Prevents coast-out between commands

## Examples

See the main `robot.py` for complete integration examples.

---

**Need help?** Check docstrings in individual modules or ask a team member.
