# V9 Swerve Basic - REV Easy Swerve Documentation

This is a basic swerve drive implementation for FRC Team 9214 using RobotPy 2026 and REV Robotics SparkMax motors with Easy Swerve modules.

## Table of Contents

1. [Hardware Setup](#hardware-setup)
2. [Architecture Overview](#architecture-overview)
3. [File Descriptions](#file-descriptions)
   - [robot.py](#robotpy---main-robot-controller)
     - [robotInit()](#robotinit-initializes-all-hardware)
     - [teleopInit()](#teleopinitcalled-when-entering-teleop-mode)
     - [teleopPeriodic()](#teleopperiodic-called-repeatedly-during-teleop)
     - [disabledInit()](#disabledinitcalled-when-robot-is-disabled)
   - [drive.py](#drivepy---swerve-drive-control)
     - [SwerveDrive Class](#swervedrive-class)
     - [SwerveModule Class](#swervemodule-class)
   - [joystick_drive.py](#joystick_drivepy---input-processing)
     - [DriverJoystick Class](#driverjoystick-class)
4. [Control Flow](#control-flow---what-happens-when-you-move-the-joystick)
5. [Special Cases](#special-cases)
6. [REV Motor API](#rev-motor-api)
7. [Debugging](#debugging)
8. [Testing Tips](#testing-tips)
9. [Future Improvements](#future-improvements)

---

## Hardware Setup

- **RoboRIO**: Main robot controller at 172.22.11.2 (team 9214)
- **Swerve Modules**: 4 modules (FL, FR, RL, RR) with SDS MK4i L2 wheels
- **Motors**: 8x REV SparkMax brushless motors
  - 4 drive motors (one per module) - CAN IDs: 2, 8, 4, 6
  - 4 turn motors (one per module) - CAN IDs: 3, 9, 5, 7
- **Wheel Specs**: 0.1016m diameter, 6.75:1 gear ratio

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│ robot.py - Main Robot Lifecycle                         │
│ - robotInit: Creates joystick and swerve drive          │
│ - teleopInit/periodic: Reads joystick, calls swerve    │
│ - disabledInit: Resets and stops all motors            │
└────────────┬────────────────────────────────────────────┘
             │
      ┌──────┴──────────────────────────────────┐
      │                                         │
      ▼                                         ▼
┌──────────────────────────┐  ┌────────────────────────────┐
│ joystick_drive.py        │  │ drive.py                   │
│ Input Handling           │  │ Swerve Kinematics & Motor  │
│ - Deadband filtering     │  │ Control                    │
│ - Axis mapping           │  │                            │
│ - Button handling        │  │ SwerveDrive class:         │
│                          │  │ - Kinematics calculation   │
│                          │  │ - Module state commands    │
│                          │  │                            │
│                          │  │ SwerveModule class:        │
│                          │  │ - Drive motor control      │
│                          │  │ - Turn motor steering      │
│                          │  │ - Reverse motion logic     │
└──────────────────────────┘  └────────────────────────────┘
```

[↑ Back to Table of Contents](#table-of-contents)

## File Descriptions

### robot.py - Main Robot Controller

**Key Methods:**

- [`robotInit()`](#robotinit-initializes-all-hardware): Initializes all hardware
  - Creates 4 SwerveModules with CAN IDs
  - Creates [SwerveDrive](#swervedrive-class) with kinematics
  - Initializes [joystick](#driverjoystick-class)
  
- [`teleopInit()`](#teleopinitcalled-when-entering-teleop-mode): Called when entering teleop mode
  - Resets all module states to 0° angle
  - Stops all motors
  - Prepares for fresh control input
  - *Called before* [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop)
  
- [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop): Called repeatedly during teleop
  - Reads [joystick inputs](#driverjoystick-class) (forward, strafe, rotation)
  - Calls [`swerve.drive()`](#drive-method) with the inputs
  - Applies slow mode multiplier if button held
  
- [`disabledInit()`](#disabledinitcalled-when-robot-is-disabled): Called when robot is disabled
  - Stops all motors
  - Resets module angle tracking to 0°
  - Prepares for next enable cycle by resetting reference angle

**Joystick Mapping:**
- Left stick Y (axis 1): Forward/backward motion
- Left stick X (axis 0): Strafe (left/right)
- Right stick X (axis 4): Rotation (spin)
- Deadband threshold: 0.1 (10% stick deflection)

See [DriverJoystick](#driverjoystick-class) for input processing details.

[↑ Back to Table of Contents](#table-of-contents)

#### robotInit(): Initializes all hardware

Called once when robot boots up. Sets up:
- 4 SwerveModules with CAN IDs (2,3,8,9,4,5,6,7)
- SwerveDrive kinematics object with module locations
- Driver joystick on port 0

→ Next called: [`teleopInit()`](#teleopinitcalled-when-entering-teleop-mode)

#### teleopInit(): Called when entering teleop mode

Called when transitioning from disabled to teleop. Resets:
- Module `current_state` to 0° angle (for fresh angle tracking)
- Stops all motors by calling `stop()` on each module
- Clears any residual motion from previous runs

→ Followed by: [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop) (runs repeatedly)
→ Back to: [`robotInit()`](#robotinit-initializes-all-hardware)

#### teleopPeriodic(): Called repeatedly during teleop

Main control loop (runs ~50 times/second):
1. Reads raw joystick values via [DriverJoystick](#driverjoystick-class)
2. Applies deadband filtering
3. Calls [`swerve.drive()`](#drive-method) with forward/strafe/rotation
4. Kinematics converts to individual [SwerveModule](#swervemodule-class) state commands

→ Calls: [`SwerveDrive.drive()`](#drive-method)
→ Calls: [`DriverJoystick.get_forward/strafe/rotation()`](#driverjoystick-class)

#### disabledInit(): Called when robot is disabled

Triggered when robot is disabled (e.g., operator disables, match ends):
- Stops all motors to ensure safe shutdown
- Resets module states to 0° angle
- Prepares angle tracking for next enable without stale references

→ Will be followed by: [`teleopInit()`](#teleopinitcalled-when-entering-teleop-mode) (if re-enabled to teleop)

[↑ Back to Table of Contents](#table-of-contents)

### drive.py - Swerve Drive Control

Manages overall swerve drive kinematics and module coordination. Created in [`robotInit()`](#robotinit-initializes-all-hardware).

##### drive() method

Main control entry point called from [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop):

```python
swerve.drive(forward_speed, strafe_speed, rotation_speed)
```

Flow:
1. Converts percentage inputs (-1 to 1) to m/s (multiplies by max_speed 4.0)
2. Creates ChassisSpeeds object from inputs
3. Runs kinematics to calculate desired wheel states
4. Normalizes wheel speeds via `desaturateWheelSpeeds()` if needed
5. Sends desired states to each [SwerveModule](#swervemodule-class) via `set_desired_state()`

→ Calls: [`SwerveModule.set_desired_state()`](#set_desired_state-method) (x4 modules)
→ Called by: [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop)

**Key Parameters:**
- `max_speed = 4.0` m/s: Maximum driving speed
- `track_width_m`, `wheelbase_m`: Module layout geometry

[↑ Back to Table of Contents](#table-of-contents)

#### SwerveModule Class

Controls a single swerve module (one drive motor + one turn motor). Created in [`robotInit()`](#robotinit-initializes-all-hardware).

##### set_desired_state() method

Called by [`SwerveDrive.drive()`](#drive-method) to command the module to move to a target state.

**Reverse Motion Optimization** (key logic):
- Kinematics returns `speed=4.0, angle=180°` for reverse motion
- This would spin wheels 180° and go forward - inefficient!
- Instead: **Detects if angle > 90°, flips speed sign and rotates angle 180°**
- Result: `speed=-4.0, angle=0°` (go backward without spinning wheels)
- Why? Avoids unnecessary wheel rotation, smoother motion

**Motor Commands:**
1. Clamps speed to [-1.0, 1.0] (motor API limit)
2. Sends speed to drive motor via `motor.set(speed_cmd)`
3. Calls [`turn_motor_steering_control()`](#turn-motor-steering) to reach desired angle

→ Called by: [`SwerveDrive.drive()`](#drive-method)
→ Calls: [`get_angle()`](#get_angle-method)

##### get_angle() method

Returns current wheel angle from the absolute encoder:

```python
current_angle = module.get_angle()  # Returns Rotation2d
```

- Reads absolute encoder position from turn motor
- Converts rotations to degrees
- Returns Rotation2d object for angle math

→ Called by: [`set_desired_state()`](#set_desired_state-method)
→ Used by: Turn motor steering proportional control

##### Turn Motor Steering

Proportional control to reach desired angle:

```python
angle_error = desired_angle - current_angle
turn_cmd = angle_error / 180.0  # Normalize to [-1, 1]
```

- At 180° error: 100% motor power
- At 90° error: 50% motor power
- At 0° error: 0% (stationary)

This ensures wheels gradually rotate to facing direction, then hold position.

→ Calls: [`get_angle()`](#get_angle-method) to get current position
→ Called by: [`set_desired_state()`](#set_desired_state-method) at end of method

**Motor Initialization** (hardware only):
- Motors only initialize if running on RoboRIO (`HAS_REV = True`)
- On Windows dev machines, motors are None and all motor commands are skipped

[↑ Back to Table of Contents](#table-of-contents)

### joystick_drive.py - Input Processing

#### DriverJoystick Class

Wraps the Xbox controller and handles input processing. Created in [`robotInit()`](#robotinit-initializes-all-hardware) and read in [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop).

**Configuration:**
- Port 0: Driver controller
- Forward axis: 1 (left stick Y, negated because down = -1.0)
- Strafe axis: 0 (left stick X)
- Rotation axis: 4 (right stick X)
- Slow mode: Button 1 (A button)
- Deadband: 0.1 (10%)

##### get_forward() method

Returns forward/backward speed with deadband applied:

1. Reads raw axis value (1 = left stick Y)
2. Applies [`_apply_deadband()`](#_apply_deadband-method)
3. Negates value (because controller forward = -1.0)
4. Returns 0 if within deadband, else raw value

→ Called by: [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop)
→ Calls: [`_apply_deadband()`](#_apply_deadband-method)
→ Used by: [`SwerveDrive.drive()`](#drive-method) as first parameter

##### get_strafe() method

Returns left/right strafe speed with deadband applied:

1. Reads raw axis value (0 = left stick X)
2. Applies [`_apply_deadband()`](#_apply_deadband-method)
3. Returns 0 if within deadband, else raw value

→ Called by: [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop)
→ Calls: [`_apply_deadband()`](#_apply_deadband-method)
→ Used by: [`SwerveDrive.drive()`](#drive-method) as second parameter

##### get_rotation() method

Returns rotation speed with deadband applied:

1. Reads raw axis value (4 = right stick X)
2. Applies [`_apply_deadband()`](#_apply_deadband-method)
3. Returns 0 if within deadband, else raw value

→ Called by: [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop)
→ Calls: [`_apply_deadband()`](#_apply_deadband-method)
→ Used by: [`SwerveDrive.drive()`](#drive-method) as third parameter

##### _apply_deadband() method

Utility method used by all input methods to filter out small stick movements:

```python
_apply_deadband(value, deadband=0.1)
```

- If `|value| < 0.1`: returns 0.0 (ignores small movements)
- Otherwise: returns value unchanged (passes through valid input)

**Why?** Joysticks naturally drift when idle (reading 0.02-0.05). Without deadband, robot creeps around. Deadband prevents this.

→ Called by: [`get_forward()`](#get_forward-method), [`get_strafe()`](#get_strafe-method), [`get_rotation()`](#get_rotation-method)

[↑ Back to Table of Contents](#table-of-contents)

## Control Flow - What Happens When You Move the Joystick

1. **Input**: Move left stick forward 50%
   - Joystick raw value: -0.5 (forward is negative on Y axis)
   - [`get_forward()`](#get_forward-method) negates it: 0.5

2. **Processing**: [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop) reads inputs
   - forward = 0.5, strafe = 0, rotation = 0
   - Calls [`swerve.drive(0.5, 0, 0)`](#drive-method)

3. **Kinematics**: [`drive()`](#drive-method) converts to m/s
   - forward_mps = 0.5 × 4.0 = 2.0 m/s
   - Creates ChassisSpeeds(2.0, 0, 0)
   - Kinematics calculates module wheel velocities
   - All 4 wheels get same forward velocity

4. **Module Control**: Each [`SwerveModule.set_desired_state()`](#set_desired_state-method) is called
   - Optimization checks angle: 0° → no flip needed
   - Speed remains 2.0 → clamped to 1.0
   - Drive motor gets: `motor.set(1.0)` (50% of max 4.0 m/s)
   - Turn motor gets: 0% (already facing forward) via [steering control](#turn-motor-steering)

5. **Motor Output**: Physical motors run
   - Drive motors spin forward
   - Turn motors are stationary
   - Robot moves forward at ~50% speed

[↑ Back to Table of Contents](#table-of-contents)

## Special Cases

### Reverse Motion

When moving backward, the kinematics initially returns `speed=4.0, angle=180°` (drive forward but wheels rotated 180°). The [optimization logic](#set_desired_state-method) detects this and converts it to `speed=-4.0, angle=0°` instead.

**Why?** This avoids spinning all 4 wheels 180° and instead just reverses the drive direction, which is much faster and smoother.

### State Reset on Disable/Enable

When you press disable or re-enable during teleop (via [`disabledInit()`](#disabledinitcalled-when-robot-is-disabled) and [`teleopInit()`](#teleopinitcalled-when-entering-teleop-mode)):

1. Module `current_state` is reset to `SwerveModuleState()` (0° angle, 0 speed)
2. All motors are set to 0.0
3. On re-enable, steering control starts fresh from 0° reference angle
4. This prevents angle tracking drift or incorrect references

### Slow Mode

Hold button 1 (A button) to enable slow mode:
- Multiplies all joystick inputs by 0.5
- Allows fine-tuned control at lower speeds
- Useful for precise positioning
- Implemented in [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop)

[↑ Back to Table of Contents](#table-of-contents)

## REV Motor API

The code uses REV Robotics SparkMax motors with these constraints:

- **`motor.set(speed)`**: Accepts -1.0 to 1.0 only
  - Values outside this range are clamped by the motor
  - Our code clamps before calling in [`set_desired_state()`](#set_desired_state-method) to maintain visibility
  
- **`motor.getAppliedOutput()`**: Returns the clamped speed (-1.0 to 1.0)
  - Used in [`set_desired_state()`](#set_desired_state-method) for debug output
  
- **`motor.getAbsoluteEncoder()`**: Gets the turn motor's absolute encoder
  - Used in [`get_angle()`](#get_angle-method) for turn motor sensing
  
- **Note**: No PID controller API, no idle mode API in 2026.0.2

[↑ Back to Table of Contents](#table-of-contents)

## Debugging

When enabled, the robot prints:
- Module initialization (in [`robotInit()`](#robotinit-initializes-all-hardware)): `"M# INIT: Drive=# Turn=#"`
- Motor commands (in [`set_desired_state()`](#set_desired_state-method) when motion > 0.01): `"M#: Speed=X.XXX Applied=X.XXX"`
- Joystick input (in [`teleopPeriodic()`](#teleopperiodic-called-repeatedly-during-teleop)): `"INPUTS - Forward: X.XXX Strafe: X.XXX Rotation: X.XXX"`
- State changes (in [`teleopInit()`](#teleopinitcalled-when-entering-teleop-mode) / [`disabledInit()`](#disabledinitcalled-when-robot-is-disabled)): `"[ROBOT] Teleop mode started"`, `"[ROBOT] Disabled state entered"`

[↑ Back to Table of Contents](#table-of-contents)

## Testing Tips

1. **Forward/Backward First**: 
   - Enable, move left stick ([`get_forward()`](#get_forward-method)) forward/backward slowly
   - All 4 drive motors should move in same direction (see [`set_desired_state()`](#set_desired_state-method))
   - Applied output should vary with stick position (not always 1.0)

2. **Strafe Motion**:
   - Move left stick left/right ([`get_strafe()`](#get_strafe-method)) while holding forward
   - Front wheels rotate 90° via [steering control](#turn-motor-steering)
   - All wheels move sideways (robot slides perpendicular to forward direction)

3. **Pure Rotation**:
   - Move right stick left/right ([`get_rotation()`](#get_rotation-method)) without moving left stick
   - All wheels rotate to diagonal (45°) via [steering control](#turn-motor-steering)
   - Robot spins in place

4. **Disable/Enable**:
   - Enable → move stick → disable (triggers [`disabledInit()`](#disabledinitcalled-when-robot-is-disabled))
   - Re-enable → wheels should reset angle reference (via [`teleopInit()`](#teleopinitcalled-when-entering-teleop-mode))
   - Next motion should work correctly (not stutter or reverse unexpectedly due to stale angle references)

[↑ Back to Table of Contents](#table-of-contents)

## Future Improvements

- [ ] Add gyro for field-relative driving
- [ ] Implement auto-balance or trajectory following
- [ ] Add intake/shooter control
- [ ] Tune steering proportional gain constant
- [ ] Handle motor faults with LED/dashboard feedback
