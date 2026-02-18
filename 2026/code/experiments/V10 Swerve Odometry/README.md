# V9 Swerve Basic - Joystick Drive Control

A simple swerve drive implementation for FRC Team 9214 using RobotPy 2026 and REV Robotics SparkMax motors.

## Table of Contents

1. [Hardware Setup](#hardware-setup)
2. [Files Overview](#files-overview)
3. [Control Flow](#control-flow)
4. [Debugging](#debugging)

---

## Hardware Setup

- **RoboRIO**: Main robot controller at 172.22.11.2 (team 9214)
- **Swerve Modules**: 4 modules (FL, FR, RL, RR)
- **Motors**: 8x REV SparkMax brushless motors
  - 4 drive motors (one per module) - CAN IDs: 2, 8, 4, 6
  - 4 turn motors (one per module) - CAN IDs: 3, 9, 5, 7

## Files Overview

### robot.py - Main Robot Controller

The main robot class that initializes hardware and runs the control loop.

**Methods:**
- `robotInit()`: Called once on startup
  - Creates 4 SwerveModules with CAN IDs
  - Creates SwerveDrive instance
  - Creates DriverJoystick on port 0

- `teleopInit()`: Called when entering teleop mode
  - Calls `swerve.stop()` to stop all motors

- `teleopPeriodic()`: Called ~50 times/second during teleop
  - Reads joystick forward and rotation inputs
  - If either input > 0: calls `swerve.drive(forward, rotation)`
  - Otherwise: calls `swerve.stop()`

- `disabledInit()`: Called when robot is disabled
  - Calls `swerve.stop()`

### drive.py - Motor Control

Handles swerve module control and motor commands.

**SwerveModule Class:**
- Controls one swerve module (drive motor + turn motor)
- Constructor: `__init__(drive_motor_id, turn_motor_id)`
- Methods:
  - `set_drive(speed)`: Sets drive motor speed (-1.0 to 1.0)
  - `set_turn(speed)`: Sets turn motor speed (-1.0 to 1.0)
  - `stop()`: Stops both motors

**SwerveDrive Class:**
- Controls all 4 swerve modules
- Constructor: `__init__(front_left, front_right, rear_left, rear_right)`
- Methods:
  - `drive(forward_speed, rotation_speed)`: Main control method
    - Applies `forward_speed × 0.3` to all drive motors
    - Applies `rotation_speed × -0.3` to all turn motors
  - `stop()`: Stops all 4 modules

### joystick_drive.py - Input Processing

Handles joystick input with deadband filtering.

**DriverJoystick Class:**
- Port: 0 (driver controller)
- Configuration:
  - FORWARD_AXIS = 1 (left stick Y)
  - ROTATION_AXIS = 4 (right stick X)
  - DEADBAND = 0.25 (25% threshold)

**Methods:**
- `get_forward()`: Returns forward/backward speed
  - Reads axis 1, negates it (forward = negative on Y axis)
  - Returns 0.0 if within deadband, otherwise raw value
  - Prints changes to console for debugging

- `get_rotation()`: Returns rotation speed
  - Reads axis 4 (right stick X)
  - Returns 0.0 if within deadband, otherwise raw value
  - Prints changes to console for debugging

Deadband filtering prevents joystick drift from causing unwanted motion.

## Control Flow

1. **Startup** → `robotInit()` creates 4 modules, SwerveDrive, and joystick
2. **Teleop mode** → `teleopInit()` stops all motors
3. **Control loop** → `teleopPeriodic()` reads joystick and calls `swerve.drive()`
4. **Motor output** → SwerveDrive applies speeds to motors:
   - All drive motors: `forward_speed × 0.3`
   - All turn motors: `rotation_speed × -0.3`
5. **Disabled** → `disabledInit()` stops all motors

## Debugging

The code prints to console:
- **Module init**: `"M<id> INIT: Drive=<id> Turn=<id>"` when each module is created
- **Joystick input**: `"Forward: X.XXX"` or `"Rotation: X.XXX"` when value changes

To see these messages:
1. Deploy code: `python -m robotpy deploy`
2. Open RioLog or Driver Station console
3. Move joystick and watch for output
