# V20 Swerve Drive System

## Overview
The V20 swerve drive is a four-wheel independent steering and driving system that enables holonomic movement (any direction with any rotation). Each wheel module can rotate independently while being driven separately, allowing the robot to move and turn in any combination.

## Architecture

### Core Components

**SwerveDrive** (`swerve_drive.py`)
- Central controller managing all four swerve modules
- Coordinates wheel angles and power across the entire drivetrain
- Maintains odometry (position tracking) and IMU (gyroscope) integration
- Handles motor current limits and power ramping for smooth acceleration

**SwerveWheel** (`swerve_wheel.py`)
- Represents a single swerve module with two motors:
  - **Drive Motor**: Controls forward/backward wheel speed
  - **Turn Motor**: Controls wheel steering angle
- Uses a duty cycle encoder to track absolute wheel angle (0-360°)
- Applies manual offsets for encoder calibration

**PIDController** (`pid_controller.py`)
- Handles wheel angle feedback control during steering movements
- One controller per wheel for precise angle targeting
- Tuned per wheel based on battery voltage and calibration data

### Wheel Configuration
Four wheels positioned at corners:
- `front_left`, `front_right`, `rear_left`, `rear_right`
- Each has CAN IDs for drive and turn motors
- Absolute encoders for reliable steering angle feedback
- Manual offsets stored in calibration

## Steering Mechanics

### Wheel Angle Control
1. **Target Angle**: Desired steering direction (0-360°)
2. **PID Feedback**: Uses encoded absolute position to reach target
3. **Shortest Path**: Selects shortest rotation (e.g., 350° → 10° becomes -10° rotation)
4. **Alignment Timeout**: 4-second maximum to reach target before timing out

### Acceleration Smoothing
- Drive power ramps down smoothly when braking (avoid jerky stops)
- Acceleration handled by higher-level systems (navigator or joystick)
- Power ramp rate: 0.2 per cycle (~10ms), smooth but responsive

## Odometry & Positioning

**SwerveOdometry** (`swerve_odometry.py`)
- Tracks robot position (X, Y in cm) and heading (0-360°)
- Accumulates wheel rotation changes to compute movement
- Blends with IMU heading for drift correction
- Dead-reckoning: useful but accumulates error over time

**SwerveIMU** (`swerve_imu.py`)
- NavX-MXP gyroscope for heading measurement
- Automatically calibrates at startup
- Provides reliable, drift-free rotation angle
- Primary source of heading (100% IMU, minimal odometry blending)

## Motor & Safety

**Current Limiting**
- Peak limit: 45A per motor
- Sustained limit: 35A per motor
- Alerts triggered when limits exceeded (for diagnostics)

**Power Ramping**
- Prevents sudden jerky movements
- Only applies to deceleration (stopping is smooth)
- Acceleration controlled by higher-level commands

## Calibration

**EncoderCalibration** (`encoder_calibration.py`)
- Stores per-wheel encoder offsets (to account for installation variations)
- Stores PID gains interpolated across battery voltage range
- Provides rotation and navigation gains for autonomous movement
- Supports live tuning via SmartDashboard

## Control Modes

**Manual Control (Teleop)**
- Driver provides desired velocities (forward, strafe, rotation)
- Swerve drive calculates required wheel angles and speeds
- Responsive to joystick input

**Autonomous Navigation**
- WaypointNavigator issues target angles and speeds
- SwerveDrive executes the commands with PID feedback
- Integrated odometry and IMU for path accuracy

## Tuning & Diagnostics

**SmartDashboard Integration**
- Live wheel angles, drive power for each wheel
- Odometry position (X, Y) and heading
- IMU status and raw yaw value
- PID tuning parameters can be adjusted in real-time
- Counter for performance monitoring

**Swerve Tuner** (`swerve_tune.py`)
- Records motor performance and PID behavior
- Helps identify optimal gains for different battery voltages
- Supports automated calibration workflows

## Key Design Decisions

1. **Absolute Encoders**: Every wheel knows its angle without needing to home at startup
2. **100% IMU for Heading**: Eliminates drift from wheel slippage and odometry errors
3. **Per-Wheel PID**: Each wheel controlled independently for precision
4. **Shortest Path Steering**: Wheels don't spin more than 90° to reach any angle
5. **Voltage-Based Gain Interpolation**: Accounts for battery sag affecting motor response
