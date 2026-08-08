# Swerve Drive Module Files

This document provides an overview of each file in the `swerve/` folder and its purpose within the swerve drive system.

## Core Modules

### `swerve_drive.py`
**Purpose**: Main orchestration class for the entire swerve drive system.

- Initializes all four swerve wheels with their motor and encoder configurations
- Manages PID controllers for each wheel's turn alignment
- Handles wheel alignment logic with timeout protection
- Coordinates odometry and IMU integration
- Manages SwerveTuner for automatic PID gain tuning
- Provides methods to drive the robot with field-relative or robot-relative movement
- Posts telemetry data to SmartDashboard

### `swerve_wheel.py`
**Purpose**: Abstraction for a single swerve module (wheel).

- Encapsulates one swerve module's drive motor, turn motor, and absolute encoder
- Handles motor initialization with error logging for CAN communication failures
- Provides methods to set drive power and turn angle
- Manages the absolute encoder reading and offset application
- Tracks current drive power for dashboard telemetry
- Applies directional corrections for each wheel position

### `swerve_config.py`
**Purpose**: Configuration constants and pin assignments.

- Defines CAN IDs for all drive and turn motors (4 wheels × 2 motors = 8 motors)
- Specifies DIO (Digital Input/Output) pins for absolute encoders
- Provides wheel positions (x, y coordinates relative to robot center)
- Defines rotation angles used for 360° in-place spin maneuvers
- Stores button assignments for manual wheel alignment testing
- Includes manual encoder offsets for mechanical mounting variations
- Contains motor control scaling factors and gear ratios

## Control & Calibration

### `pid_controller.py`
**Purpose**: General-purpose PID (Proportional-Integral-Derivative) controller.

- Implements standard PID control loop mathematics
- Calculates proportional, integral, and derivative terms
- Prevents integral windup with maximum accumulation limits
- Tracks timing between updates for accurate derivative calculation
- Logs debug information for tuning analysis
- Used by swerve drive to control wheel turn angle alignment

### `encoder_calibration.py`
**Purpose**: Manages persistent calibration data storage and retrieval.

- Loads and saves encoder offset calibrations to a JSON file on the RoboRIO
- Stores PID tuning history with battery voltage tracking
- Manages interpolated PID gains based on battery voltage (to handle voltage sag)
- Calculates regression fits for voltage-dependent PID behavior
- Handles rotation alignment gains for navigator-based path following
- Provides methods to retrieve appropriate gains for current operating conditions

## Sensing & State Tracking

### `swerve_odometry.py`
**Purpose**: Position tracking via dead reckoning using wheel encoders.

- Tracks robot pose (x, y position and heading) in field coordinates
- Calculates position from drive motor encoder counts and wheel angles
- Converts robot-relative movement to field coordinates using robot heading
- Supports position correction from external sensors (e.g., AprilTag camera)
- Tracks total distance driven for distance-based movement commands
- Uses REV EasySwerve (REV-21-3006) hardware constants (2.1:1 gear ratio, 4" wheel)

### `swerve_imu.py`
**Purpose**: NavX2 MXP IMU wrapper for precise heading measurement.

- Interfaces with NavX2 sensor via MXP SPI connection
- Waits for self-calibration on power-up (~1-2 seconds)
- Converts NavX's native yaw range (-180..180) to 0-360 degrees
- Follows field-standard coordinate convention (CCW positive)
- Handles IMU inversion for alternative mounting orientations
- Provides zeroing capability to align heading with field reference
- Posts heading and raw gyro data to SmartDashboard

## Advanced Features

### `swerve_tune.py`
**Purpose**: Automatic PID gain tuning system.

- Implements relay auto-tuning for robust gain discovery
- Tests wheels at multiple angles (0°, 90°, 180°, 270°)
- Uses critical gain (Kc) and critical period (Tc) to calculate PID coefficients
- Tracks oscillation and sign changes to determine stability
- Stores tuning results with battery voltage for later interpolation
- Provides safe tuning boundaries to prevent unsafe motor speeds

### `catmull_rom.py`
**Purpose**: Smooth spline interpolation for path following.

- Implements Catmull-Rom spline evaluation through waypoint sequences
- Generates smooth curves between waypoints with automatic tangent calculation
- Supports parametric evaluation along the spline (segment and t parameter)
- Returns interpolated position (x, y) in centimeters
- Can be extended to support heading interpolation
- Used by navigator and autonomous path planning

## Package Initialization

### `__init__.py`
**Purpose**: Python package marker and optional module imports.

- Makes the `swerve/` folder a valid Python package
- Can include convenient re-exports of main classes (SwerveDrive, SwerveWheel, etc.)

---

## Module Dependencies

**Dependency Flow:**
```
swerve_drive.py (main orchestrator)
  ├── swerve_wheel.py (individual modules)
  ├── pid_controller.py (turn alignment)
  ├── encoder_calibration.py (stored gains & offsets)
  ├── swerve_odometry.py (position tracking)
  ├── swerve_imu.py (heading measurement)
  ├── swerve_tune.py (auto-tuning)
  ├── catmull_rom.py (path splines)
  └── swerve_config.py (shared configuration)
```

## Typical Usage Flow

1. **Initialization**: `SwerveDrive.__init__()` creates wheels, loads calibrations
2. **Telemetry**: PID gains interpolated based on current battery voltage
3. **Control**: `drive_for_velocity()` or `drive_for_heading()` commands movement
4. **Feedback Loop**:
   - PIDController adjusts wheel angles toward setpoint
   - SwerveOdometry accumulates distance and position from encoder counts
   - SwerveIMU provides absolute heading for field-relative commands
5. **Tuning**: SwerveTuner can auto-discover PID gains if in-field tuning is triggered
6. **Path Following**: CatmullRomSpline generates smooth reference trajectory

---

## Configuration & Calibration Files

- **Encoder offsets** and **PID gains** are stored on the RoboRIO filesystem
- Location defined by `OFFSET_FILE` in `swerve_config.py`
- Gains are voltage-dependent to handle battery sag during matches
- Manual offsets in config account for physical mounting variations
- Tuning history allows regression analysis of gain relationships
