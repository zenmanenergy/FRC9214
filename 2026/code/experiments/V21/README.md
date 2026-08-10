# FRC 9214 Team - V21 Swerve Drive Experiment

A production-ready swerve drive system with field-relative control, autonomous path following, and adaptive tuning.

## Overview

This is Team 9214's **V21 experimental swerve drive** featuring:

- **4-wheel swerve modules** - Each wheel can rotate and drive independently
- **Field-relative movement** - Drive using field coordinates, not robot-relative
- **Odometry & IMU fusion** - Dead reckoning with gyro sensor fusion for accurate positioning
- **Adaptive PID tuning** - Auto-calibrated wheel control with battery voltage compensation
- **Autonomous paths** - Catmull-Rom spline path following with smooth acceleration
- **Production library** - Type-hinted, documented, reusable swerve module
- **Collision detection** - Motor current monitoring for impact safety

## Quick Start

```python
from swerve import SwerveDrive

# Initialize swerve drive system
swerve = SwerveDrive()

# Field-relative drive (forward, strafe, rotate)
swerve.drive_swerve(forward=0.5, strafe=0.2, rotate=0.1)

# Update odometry and sensor fusion
swerve.odometry.update()
swerve.imu.fuse_heading(swerve.odometry)

# Get current position
x, y = swerve.odometry.get_position()
heading = swerve.odometry.get_heading()
print(f"Position: ({x:.1f}, {y:.1f}) @ {heading:.1f}°")
```

## Folder Structure

```
V21/
├── robot.py                    Main robot entry point
├── swerve/                     Production swerve drive library
│   ├── __init__.py            Public API
│   ├── swerve_drive.py        Main drive coordinator
│   ├── swerve_wheel.py        Individual wheel module
│   ├── swerve_odometry.py     Dead reckoning from encoders
│   ├── swerve_imu.py          NavX2 gyro integration
│   ├── pid_controller.py      PID control & autotuning
│   ├── encoder_calibration.py Persistent offset storage
│   ├── swerve_tune.py         Automated tuning system
│   ├── catmull_rom.py         Smooth path following
│   ├── swerve_config.py       Hardware configuration
│   └── README.md              Swerve library docs
├── dashboard/                 Web control interface
├── tests/                     Unit & integration tests
├── pilot_controls.py          Joystick input handling
├── waypoint_navigator.py      Autonomous navigation
└── networktables.json         Configuration file
```

## Core Components

| Component | Purpose |
|-----------|---------|
| `SwerveDrive` | Coordinates all wheels and subsystems |
| `SwerveWheel` | Individual wheel with drive + turn motor |
| `SwerveOdometry` | Position tracking from wheel encoders |
| `SwerveIMU` | Gyro fusion for heading correction |
| `PIDController` | Smooth wheel alignment & autotuning |
| `CatmullRomSpline` | Smooth autonomous path curves |
| `EncoderCalibration` | Persistent tuning & offset storage |

## Configuration

Edit `swerve/swerve_config.py` to match your robot:

```python
WHEELS = {
    "front_right": {
        "drive_canid": 10,      # CAN ID of drive motor
        "turn_canid": 11,       # CAN ID of turn motor
        "encoder_dio": 0,       # DIO port for encoder
        "position": {"x": 0.5, "y": 0.5},  # Relative to center
    },
    # ... other wheels
}

ROBOT_TRACKWIDTH_CM = 56.0   # Left-to-right distance
ROBOT_WHEELBASE_CM = 53.0    # Front-to-rear distance
```

## Drive Modes

### Teleop (Field-Relative)
```python
swerve.drive_swerve(forward=joy_y, strafe=joy_x, rotate=joy_z)
```

### Straight Line
```python
swerve.drive_straight(speed=0.5, target_angle=90.0)  # 90° heading
```

### Distance-Limited
```python
done = swerve.drive_for_distance(speed=0.5, target_distance_cm=200.0)
```

### In-Place Rotation
```python
done = swerve.drive_to_heading(target_angle=180.0)
```

## Autonomous Navigation

```python
# Define waypoints
waypoints = [
    {'x': 0, 'y': 0, 'heading': 0},
    {'x': 100, 'y': 50, 'heading': 45},
    {'x': 200, 'y': 100, 'heading': 90},
]

# Start following the path
swerve.follow_path(waypoints, speed=0.6)

# In main autonomous loop
while not swerve.is_path_complete():
    swerve.update_autonomous()
    # Path following, odometry, and heading fusion all handled automatically
```

## Calibration & Tuning

### Encoder Offsets
Wheel angles are calibrated to absolute encoder readings. Calibration data persists in `/home/lvuser/encoder_offsets.json`.

### PID Autotuning
The system auto-tunes wheel PID gains using relay method with battery voltage correlation:

```python
tuner = swerve.tuner
tuner.start()  # Begin autotuning sequence

# In main loop:
while tuner.is_active():
    tuner.update()
    # Gains are saved automatically when complete
```

## Motor Configuration

- **Drive motors**: SparkMax brushless (coast mode by default)
- **Turn motors**: SparkMax brushless (coast mode by default)
- **Encoders**: DutyCycleEncoder for absolute wheel angles
- **Gyro**: NavX2 on MXP SPI port

To enable brake mode on motors, modify `swerve/swerve_wheel.py`.

## Performance

- **Max speed**: ~15 ft/s (configurable via `MOTOR_SCALE_TELEOP`)
- **Rotation speed**: ~360°/s in-place
- **Alignment tolerance**: ±5° (configurable)
- **Current monitoring**: Collision detection at 45A spike / 35A sustained

## Testing

Run unit tests:
```bash
python -m pytest tests/
```

Dashboard available at: `http://roboRIO-9214-frc.local:5800`

## Documentation

- [Swerve Library API](swerve/README.md) - Detailed component documentation
- Main code entry point: `robot.py`

---

**Team 9214 - Build Season 2026**
