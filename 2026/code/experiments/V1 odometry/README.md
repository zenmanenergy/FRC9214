# FRC 9214 V1 Odometry Robot Code

## Overview
This is a FIRST Robotics Competition (FRC) robot program that implements a tank drive system with odometry tracking. The robot can track its position on the field and execute autonomous driving commands.

## Components

### Main Robot Class (`MyRobot`)
Extends `wpilib.TimedRobot` to create the main robot controller that runs periodic update loops during operation.

### Hardware
- **Drive Motors**: Four WPI_TalonSRX motor controllers
  - Left front (ID 4) and rear (ID 2)
  - Right front (ID 3) and rear (ID 1)
  - Right-side motors are inverted to ensure both sides move forward together
  - All motors set to brake mode to prevent coasting

- **Encoders**: REV through-bore encoders for odometry
  - Left encoder on DIO ports 0-1
  - Right encoder on DIO ports 8-9
  - Wheel diameter: 15.24 cm

- **Controller**: Joystick (port 1) for teleop control

### Core Modules
- **Drive**: Handles motor control and movement commands
- **Odometry**: Tracks robot position and heading using encoder feedback

## Functionality

### Initialization (`robotInit`)
Sets up all motors, encoders, joystick input, and initializes the Drive and Odometry systems.

### Disabled State (`disabledInit`)
Resets the drive system and clears the zeroed state when the robot is disabled.

### Teleop Mode (`teleopInit` and `teleopPeriodic`)
- **teleopInit**: Resets odometry and drive on first teleop run
- **teleopPeriodic**: Updates every ~20ms and handles:
  - Drive control via joystick
  - Odometry position tracking
  - **A Button**: Resets odometry to (0, 0)
  - **B Button**: Autonomously drives to position (100, 0)
  - Debug printing (commented out, prints every ~1 second)

## Usage
Run with: `python robot.py`

The robot will connect to the FRC field management system and wait for enable commands from the driver station.
