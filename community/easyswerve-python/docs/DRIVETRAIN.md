# EasySwerve Python Template - Reference and Guide

> **Coach reference.** This guide was written before the project was simplified, so some file names below are old. Where things are now:
>
> | Old name in this guide | Now |
> |---|---|
> | constants.py: CAN IDs, DIO ports, motor mounting, sizes, encoder location and version, gyro, steering zero backup (kSteeringEncoderOffsetsRad, now STEERING_ZEROS_DEG in degrees) | robot_settings.py |
> | constants.py: everything else | easyswerve/drivetrain/constants.py |
> | configs.py, subsystems/ | easyswerve/drivetrain/ |
> | tuning/ | easyswerve/tuning/ |
> | physics.py settings (SIM_...) | easyswerve/sim/physics.py |
> | easyswerve_pathplanner, EasySwervePathPlanner | easyswerve/autos/ (robotcontainer.py uses Autos) |
> | easyswerve_vision, its config.py cameras and VISION_ENABLED | easyswerve/vision/; cameras and VISION_ENABLED are in robot_settings.py |
> | self.drivetrain, DriveSubsystem() | self.drivetrain, Drivetrain() |
> | robotcontainer.py, RobotContainer | robot.py (built in robotInit; no separate container) |
> | examples/robotcontainer.py, examples/deploy/pathplanner/ | examples/<name>/robot.py: 34 small examples (examples/README.md) |
>
> Autotune and calibration buttons are created by Drivetrain() automatically.


RobotPy 2026 port of REV's EasySwerve-Java-Template v2026.0.

Hardware this code assumes:

- 4 x REV 4in EasySwerve Modules (101.6 mm wheels, 6.3:1 drive, 20:1 steering)
- 8 x SPARK MAX (one drive, one turn per module)
- 8 x NEO 2.0
- 4 x REV Through Bore Encoder, V1 or V2 (set kThroughBoreEncoderVersion), wired to roboRIO DIO ports (default) or to the turning SPARK MAX data port (set kSteeringEncoderLocation)
- navX2-MXP on the roboRIO MXP port (SPI)

All lengths are meters, angles are radians unless a function says degrees, speeds are m/s and rad/s.

---

## Contents

1. Project layout
2. Install, simulate, deploy
3. Configuration (constants.py and configs.py)
4. Calibration and first power-up
5. API reference
6. Using it in your codebase (examples)
7. Troubleshooting
8. Simulation and automated tests
9. Autotune

---

## 1. Project layout

```
easyswerve-python/
  robot.py                     MyRobot - entry point, runs the command scheduler
  robotcontainer.py            RobotContainer - controller, bindings, auto command
  constants.py                 every tunable number (CAN IDs, geometry, speeds)
  configs.py                   SPARK MAX configs for drive and turn motors
  subsystems/
    drivesubsystem.py          DriveSubsystem - the whole drivetrain
    easyswervemodule.py        EasySwerveModule - one corner module
    steeringencoder.py         steering angle from a roboRIO DIO or SPARK encoder
  tuning/
    autotune.py                Autotuner - the dashboard autotune commands
    fit.py                     the math: motor model fit, delay, gain rules
    gains.py                   TunedGains and the saved tuning file
    calibration.py             saved steering encoder zeros (roboRIO wiring)
  physics.py                   simulation model (sim and tests only)
  tests/pyfrc_test.py          built-in RobotPy sim tests
  tests/test_drive_sim.py      drivetrain behavior tests against physics.py
  tests/test_autotune.py       autotune math tests and a full autotune in sim
  tests/test_steering_encoder.py  roboRIO encoder sync, drift, unplugged and stuck cases
  pyproject.toml               RobotPy version and libraries
```

How the pieces connect:

```
MyRobot
  -> RobotContainer
       -> DriveSubsystem
            -> 4 x EasySwerveModule  -> configs.drivingConfig / turningConfig
            -> navX2 (AHRS)
            -> SwerveDrive4PoseEstimator (wheels + gyro, plus cameras if the vision add-on is used)
```

---

## 2. Install, simulate, deploy

From the project folder on the laptop:

```bash
# one time: install RobotPy for your laptop
pip install robotpy

# download everything listed in pyproject.toml (laptop + roboRIO packages)
python -m robotpy sync

# run in the simulator (GUI opens)
python -m robotpy sim

# run the built-in tests (disabled, auto, teleop in sim)
python -m robotpy test

# deploy to the roboRIO (connected to the robot network)
python -m robotpy deploy
```

pyproject.toml pins the versions:

```toml
[tool.robotpy]
robotpy_version = "2026.2.2"
components = ["commands2"]
requires = [
    "robotpy-rev==2026.0.4",
    "robotpy-navx==2026.0.1.2",
]
```

RobotPy 2026 installs vendor libraries (REV, navX) through requires, not components.

---

## 3. Configuration

### 3.1 constants.py

#### DriveConstants

| Name | Default | What it is |
|---|---|---|
| kMaxSpeedMetersPerSecond | about 4.79 | top speed full stick maps to. Set to the wheel free speed. Lower it for a slower robot. |
| kMaxAngularSpeed | 2 pi | top spin rate full stick maps to (rad/s) |
| kTrackWidth | 0.6858 | left-to-right distance between wheel centers (m) |
| kWheelBase | 0.6858 | front-to-back distance between wheel centers (m) |
| kDriveKinematics | built from the two above | module positions, order FL, FR, RL, RR |
| kEasySwerveAngularOffsetCompensation | pi / 4 | EasySwerve's zero pocket sits 45 degrees from MAXSwerve's. Do not change. |
| kFrontLeftChassisAngularOffset | -pi/2 + pi/4 | where each module's encoder zero points relative to the robot |
| kFrontRightChassisAngularOffset | 0 + pi/4 | |
| kRearLeftChassisAngularOffset | pi + pi/4 | |
| kRearRightChassisAngularOffset | pi/2 + pi/4 | |
| k...DrivingMotorOnBottom | True | set False for any module whose drive NEO is mounted on top |
| k...TurningMotorOnBottom | True | set False for any module whose turn NEO is mounted on top |
| kFrontLeftDrivingCanId | 9 | SPARK MAX CAN IDs, change to match your robot |
| kFrontLeftTurningCanId | 8 | |
| kFrontRightDrivingCanId | 11 | |
| kFrontRightTurningCanId | 10 | |
| kRearLeftDrivingCanId | 15 | |
| kRearLeftTurningCanId | 14 | |
| kRearRightDrivingCanId | 13 | |
| kRearRightTurningCanId | 12 | |
| kFrontLeftSteeringEncoderDio | 0 | roboRIO DIO channel of each steering encoder (with "rio") |
| kFrontRightSteeringEncoderDio | 1 | |
| kRearLeftSteeringEncoderDio | 2 | |
| kRearRightSteeringEncoderDio | 3 | |
| STEERING_ZEROS_DEG (robot_settings.py) | None | backup steering zeros in degrees, used only if the saved zeros on the roboRIO are missing |
| kGyroReversed | False | flips getTurnRate() only |
| kOdometryStdDevs | (0.05, 0.05, 0.01) | how closely the wheels and gyro are trusted (x m, y m, heading rad); smaller = cameras correct more slowly |
| kDefaultVisionStdDevs | (0.9, 0.9, 0.9) | trust for a camera reading given without its own (the vision add-on always gives its own) |

The chassis angular offsets assume each module was zeroed with the hex key (section 4.1) and is bolted in the standard REV orientation. If you zero correctly and the wheels still point the wrong way, see Troubleshooting.

#### ModuleConstants

| Name | Value | Notes |
|---|---|---|
| kThroughBoreEncoderVersion | 1 | 1 = original Through Bore (REV-11-1271), 2 = V2 (REV-11-3174) |
| kNominalVoltage | 12.0 | voltage compensation: SPARK output is scaled so commands mean the same volts on any battery |
| kDriveVelocityMeasurementPeriodMs | 16 | drive speed averaging window (REV default 32; allowed 8 to 64) |
| kDriveVelocityAverageDepth | 2 | drive speed averaging depth (REV default 8; allowed 1, 2, 4, 8) |
| kSteerFrictionDeadbandDeg | 0.25 | steering friction push turns off inside this error |
| kDrivingMotorPinionTeeth | 12 | EasySwerve only accepts a 12T pinion |
| kWheelDiameterMeters | 0.1016 | measure a worn wheel and update for better odometry |
| kDrivingMotorReduction | 6.3 | REV's published drive ratio (REV's Java template's 7.5 is wrong) |
| kDriveWheelFreeSpeedRps | about 4.79 | wheel free speed in m/s (REV's name says Rps) |
| kSteeringEncoderLocation | "rio" | "rio" = encoders on roboRIO DIO ports, "spark" = on the turning SPARK data port |
| kSteeringReduction | 20.0 | steering motor turns per module turn (used with "rio") |
| kSteeringResyncThresholdDeg | 1.0 | with "rio": how far the NEO sensor may drift from the absolute encoder before it is corrected (wheel still) |

#### OIConstants

| Name | Default | Notes |
|---|---|---|
| kDriverControllerPort | 0 | Driver Station USB slot |
| kDriveDeadband | 0.1 | stick deadband, 0 to 1 |

#### AutoConstants

| Name | Default | Notes |
|---|---|---|
| kMaxSpeedMetersPerSecond | 3 | trajectory speed limit |
| kMaxAccelerationMetersPerSecondSquared | 3 | trajectory acceleration limit |
| kMaxAngularSpeedRadiansPerSecond | pi | heading controller limit |
| kMaxAngularSpeedRadiansPerSecondSquared | pi | heading controller limit |
| kPXController, kPYController | 1 | path following position gains |
| kPThetaController | 1 | path following heading gain |

### 3.2 configs.py

Two public functions build a fresh SparkMaxConfig. EasySwerveModule calls them. The gains come from autotune (section 9); you don't edit numbers here.

```python
configs.drivingConfig(inverted: bool, gains: TunedGains = None) -> SparkMaxConfig
configs.turningConfig(inverted: bool, gains: TunedGains = None) -> SparkMaxConfig
```

What each sets:

| Setting | Drive motor | Turn motor |
|---|---|---|
| Idle mode | brake | brake |
| Smart current limit | 50 A | 20 A |
| Feedback sensor | built-in NEO encoder | Through Bore (absolute) |
| Position units | meters | radians |
| Velocity units | m/s | rad/s |
| PID | P from autotune (default 0.04) | P from autotune (default 1) |
| Feedforward | kS and kV from autotune (default kS 0, kV 12 / free speed, about 2.50 V per m/s) | none |
| Status signals | position and speed every 20 ms | absolute angle and speed every 20 ms |
| Voltage compensation | kNominalVoltage (12 V) | kNominalVoltage (12 V) |
| Speed measurement | 16 ms window, depth 2 (about 15 ms lag instead of about 130 ms) | - |
| Friction push | - | steerKS volts in the direction of the error, sent with each setpoint (from autotune) |
| Output range | -1 to 1 | -1 to 1 |
| Wrapping | off | on, 0 to 2 pi |
| Encoder preset | - | REV_ThroughBoreEncoder (V1) or REV_ThroughBoreEncoderV2 (V2), from kThroughBoreEncoderVersion |

Module-level constants also exported by configs.py:

| Name | Value |
|---|---|
| kDrivingFactor | meters of travel per drive motor revolution (about 0.05066) |
| kTurningFactor | 2 pi (radians per encoder revolution) |
| kNominalVoltage | 12.0 |
| kDrivingVelocityFeedForward | 12 / kDriveWheelFreeSpeedRps |

Choosing the encoder model is a constant, not a code edit (constants.py):

```python
class ModuleConstants:
    kThroughBoreEncoderVersion = 1   # 1 = REV-11-1271 (original), 2 = REV-11-3174 (V2)
```

configs.throughBoreEncoderPreset(version) turns that number into REVLib's preset and raises a ValueError at startup for anything other than 1 or 2.

| | V1 (REV-11-1271) | V2 (REV-11-3174) |
|---|---|---|
| Quadrature | 2048 cycles = 8192 counts per rev | 2048 cycles = 8192 counts per rev |
| Absolute pulse period | 1025 us | 1000 us |
| Absolute pulse range | 1 to 1024 us | about 3.9 to 998 us |
| Absolute resolution | 10-bit | 12-bit |

The steering loop uses only the absolute (pulse width) signal, so the pulse timing is what the setting changes.

Changing the gyro connection (drivesubsystem.py):

```python
self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)   # navX2-MXP on MXP (default)
self.gyro = navx.AHRS(navx.AHRS.NavXComType.kUSB1)      # navX2 on a roboRIO USB port
```

---

## 4. Calibration and first power-up

Do these in order the first time, and redo 4.1 any time a module is taken apart.

### 4.1 Wire and zero the steering encoders

#### Encoders on the roboRIO (kSteeringEncoderLocation = "rio", the default)

Wiring, for each module, with the Through Bore Encoder's absolute (pulse) output:

| Encoder wire | roboRIO DIO port pin |
|---|---|
| absolute / PWM signal | S (signal) |
| 5 V | V (5 V) |
| ground | ground |

Put each module's DIO channel in DriveConstants (defaults FL 0, FR 1, RL 2, RR 3). The dashboard shows Drive/FL absolute deg and friends, so you can turn each wheel by hand and check that the right number moves. If two move together or the wrong one moves, the channels are swapped.

How it works: the roboRIO reads the true angle, but the SPARK can't use a roboRIO sensor for its fast built-in steering loop. So the SPARK steers on the NEO's own sensor (through the 20:1 steering gearing), and the code sets that sensor to the true angle at startup, keeps it set while disabled, and corrects any drift (backlash) when a wheel is still. The robot will not drive until every module has been read once (Drive/Steering encoders ready on the dashboard).

Zeroing (once, and again whenever a module or encoder is taken apart):

1. Deploy the code and connect the Driver Station. Leave the robot disabled.
2. For each module, push a 3 mm hex key into the calibration hole on the top cover until it touches the azimuth gear, then turn the wheel by hand with light pressure until the key drops about 6 mm deeper into the alignment pocket. Leave all four keys in.
3. On the dashboard, press Calibrate/Zero steering encoders (hex keys in, disabled).
4. Calibrate/Status should say "zeroed and saved". Remove the hex keys.

The zeros are saved on the roboRIO (/home/lvuser/easyswerve-calibration.json), outside the deploy folder, and loaded at every startup. The console also prints a line like

```
[drive] backup: paste this line into robot_settings.py:
STEERING_ZEROS_DEG = {"FL": 63.03, "FR": 229.18, "RL": 143.24, "RR": 338.04}
```

Paste that line into robot_settings.py, replacing STEERING_ZEROS_DEG = None. It is used only if the saved file is ever lost (for example after reimaging the roboRIO).

#### Encoders on the SPARK data port (kSteeringEncoderLocation = "spark")

For each of the 4 modules:

1. Plug that module's turning SPARK MAX into the laptop with the USB-C cable.
2. In REV Hardware Client 2, select the SPARK under Connected Devices.
3. Update tab: update firmware if offered (needs v26.1.0 or newer).
4. Utilities tab: open Absolute Encoder.
5. Push a 3 mm hex key into the calibration hole on the top cover until it touches the azimuth gear.
6. Turn the wheel by hand with light pressure on the key until the key drops about 6 mm deeper into the alignment pocket.
7. Click Zero Encoder (some versions label it Set Zero Offset).
8. Remove the hex key.

The zero offset is saved on the SPARK MAX itself.

### 4.2 Set CAN IDs

In Hardware Client 2, give every SPARK a unique CAN ID and copy them into DriveConstants. Defaults are drive 9, 11, 15, 13 and turn 8, 10, 14, 12 for FL, FR, RL, RR.

### 4.3 Tell the code where the motors are mounted

For each module, set the matching k...DrivingMotorOnBottom and k...TurningMotorOnBottom. This only changes motor inversion; getting it wrong makes that module drive backward or fight its steering.

### 4.4 Measure the chassis

Measure wheel center to wheel center:

- kTrackWidth: left wheels to right wheels
- kWheelBase: front wheels to back wheels

Wrong values make the robot drift or spiral when it translates and rotates at the same time.

### 4.5 navX2

- Mount it flat. The code reads yaw (rotation about vertical).
- After power-on, keep the robot still for about 20 seconds so it can calibrate.
- The code uses getRotation2d(), which is already counterclockwise-positive for WPILib. Do not swap in getAngle() or getYaw(); those are clockwise-positive and break field-relative driving.

### 4.6 First-drive checks (wheels off the ground)

Put the robot on blocks, enable teleop, then:

| Test | Expected | If wrong |
|---|---|---|
| Disabled, look at wheels after enabling with sticks centered | all 4 wheels point straight forward | redo 4.1 on the bad module |
| Push left stick forward | all wheels spin so the robot would go forward | that module's DrivingMotorOnBottom is wrong |
| Push left stick left | wheels turn 90 degrees, robot would go left | turn inversion wrong (TurningMotorOnBottom) or offsets wrong |
| Right stick left | wheels form a circle, robot would spin counterclockwise | check navX direction and module offsets |
| Hold right bumper | wheels form an X | normal |
| Rotate the robot by hand, then push stick forward (field-relative) | wheels point toward the far field wall, not robot-front | navX not calibrated or wrong com type |
| Press Start | heading zeroes, current robot front becomes field forward | normal |

### 4.7 Tuning

Run autotune (section 9). Nothing is tuned by hand. Until autotune has run, the robot drives on REV's default gains, which are safe but slower.

---

## 5. API reference

### 5.1 DriveSubsystem (subsystems/drivesubsystem.py)

A commands2 Subsystem. Create one, in RobotContainer.

```python
from subsystems.drivesubsystem import DriveSubsystem
drive = DriveSubsystem()
```

| Method | Returns | What it does |
|---|---|---|
| drive(xSpeed, ySpeed, rot, fieldRelative) | None | Main teleop call. xSpeed forward, ySpeed left, rot counterclockwise, each -1 to 1 (scaled by kMaxSpeedMetersPerSecond and kMaxAngularSpeed). fieldRelative True means forward is away from the driver station. |
| setModuleStates(desiredStates) | None | Low-level. Takes 4 SwerveModuleState objects (FL, FR, RL, RR) in m/s and radians, desaturates them, and sends them. Used by path following. |
| setX() | None | Points wheels in an X to resist being pushed. Call every loop while held. |
| getPose() | Pose2d | Current position estimate (m, radians): wheels and gyro, plus cameras when the vision add-on feeds them in. |
| resetOdometry(pose) | None | Sets the position without touching the gyro. Call at the start of an auto. |
| addVisionMeasurement(pose, timestampSeconds, stdDevs=None) | None | Blends a camera position (taken at timestampSeconds, FPGA time) into the estimate with the given trust (x m, y m, heading rad). |
| zeroHeading() | None | Resets navX yaw to 0; current robot front becomes field forward. |
| getHeading() | float | Heading in degrees, counterclockwise-positive. |
| getTurnRate() | float | Turn rate in degrees/second, counterclockwise-positive. |
| resetEncoders() | None | Zeroes the 4 drive encoders (distance only; steering is absolute). |
| zeroSteeringEncoders() | str | roboRIO wiring: with the hex keys in and the robot disabled, saves the current readings as the steering zeros and applies them. Returns a status message. |
| steeringEncodersReady() | bool | True once every module's steering angle has been read; driving is blocked until then. |
| periodic() | None | Called automatically every 20 ms by the scheduler; updates odometry and keeps each steering friction push aimed correctly. |

Public attributes:

| Attribute | Type |
|---|---|
| frontLeft, frontRight, rearLeft, rearRight | EasySwerveModule |
| gyro | navx.AHRS |
| poseEstimator | SwerveDrive4PoseEstimator (also reachable as odometry, the old name) |
| visionMeasurementCount | int: camera readings blended in so far |
| poseResetCount | int: times resetOdometry was called (the simulator watches it) |

### 5.2 EasySwerveModule (subsystems/easyswervemodule.py)

One corner. DriveSubsystem creates all four; you rarely make one yourself.

```python
EasySwerveModule(drivingCANId, turningCANId, chassisAngularOffset,
                 drivingMotorOnBottom, turningMotorOnBottom,
                 gains=None, name="", steeringEncoderDio=None, steeringEncoderOffset=0.0)
```

Construction configures both SPARKs (reset safe parameters, then persist) and zeroes the drive encoder.

| Method | Returns | What it does |
|---|---|---|
| getState() | SwerveModuleState | wheel speed (m/s) and angle relative to the robot |
| getPosition() | SwerveModulePosition | distance driven (m) and angle relative to the robot |
| setDesiredState(state) | None | applies the chassis offset, optimizes so the wheel never turns more than 90 degrees, then commands velocity and position (plus the steering friction push) on the SPARKs |
| resetEncoders() | None | drive distance back to 0 |
| updateSteeringFeedforward() | None | re-aims the steering friction push at the current error; DriveSubsystem.periodic() calls it every loop |

Public attributes:

| Attribute | Type | Use |
|---|---|---|
| drivingSpark, turningSpark | rev.SparkMax | current, temperature, faults |
| drivingEncoder | relative encoder | getPosition() m, getVelocity() m/s |
| turningEncoder | RioSteeringEncoder or SparkSteeringEncoder | getPosition() rad (0 to 2 pi), before chassis offset; the same interface for both wirings (subsystems/steeringencoder.py) |
| drivingClosedLoopController, turningClosedLoopController | rev.SparkClosedLoopController | direct setpoints |
| chassisAngularOffset | float | radians |
| desiredState | SwerveModuleState | last command received (before offset) |
| turningTarget | float or None | last steering setpoint, encoder frame, radians |
| turningFeedforward | float | steering friction push currently applied, volts |
| gains | TunedGains | gains currently applied to this module's SPARKs |

### 5.3 RobotContainer (robotcontainer.py)

| Member | What it is |
|---|---|
| robotDrive | the DriveSubsystem |
| driverController | wpilib.XboxController on port 0 |
| configureButtonBindings() | right bumper = X-lock while held, Start = zero heading |
| getAutonomousCommand() | example S-curve trajectory, 3 m forward |

Default command: left stick translates (field-relative), right stick X rotates.

### 5.4 MyRobot (robot.py)

commands2.TimedCommandRobot. Builds RobotContainer in robotInit, schedules the auto command in autonomousInit, cancels it in teleopInit, cancels everything in testInit.

---

## 6. Using it in your codebase

All examples go in robotcontainer.py unless noted. Imports used across examples:

```python
import commands2
import wpilib
import wpimath
from commands2.button import JoystickButton
from wpimath.geometry import Pose2d, Rotation2d
```

### 6.1 Default teleop drive (already in the template)

```python
self.drivetrain.setDefaultCommand(
    commands2.RunCommand(
        lambda: self.drivetrain.drive(
            -wpimath.applyDeadband(self.driverController.getLeftY(), 0.1),
            -wpimath.applyDeadband(self.driverController.getLeftX(), 0.1),
            -wpimath.applyDeadband(self.driverController.getRightX(), 0.1),
            True,
        ),
        self.drivetrain,
    )
)
```

The minus signs are needed because pushing a stick forward or left gives a negative value.

### 6.2 Slow mode on a trigger

```python
def _scale(self) -> float:
    # left trigger fully pressed = 30 % speed
    return 1.0 - 0.7 * self.driverController.getLeftTriggerAxis()

# in __init__, replace the default command:
self.drivetrain.setDefaultCommand(
    commands2.RunCommand(
        lambda: self.drivetrain.drive(
            -wpimath.applyDeadband(self.driverController.getLeftY(), 0.1) * self._scale(),
            -wpimath.applyDeadband(self.driverController.getLeftX(), 0.1) * self._scale(),
            -wpimath.applyDeadband(self.driverController.getRightX(), 0.1) * self._scale(),
            True,
        ),
        self.drivetrain,
    )
)
```

### 6.3 Toggle field-relative / robot-relative

```python
# in __init__
self.fieldRelative = True

# in configureButtonBindings
JoystickButton(self.driverController, wpilib.XboxController.Button.kBack).onTrue(
    commands2.InstantCommand(self._toggleFieldRelative)
)

def _toggleFieldRelative(self) -> None:
    self.fieldRelative = not self.fieldRelative

# and pass self.fieldRelative instead of True in the default command lambda
```

### 6.4 Square up the stick response (finer control near center)

```python
import math

def _shape(value: float) -> float:
    v = wpimath.applyDeadband(value, 0.1)
    return math.copysign(v * v, v)

# use -_shape(self.driverController.getLeftY()) etc. in drive()
```

### 6.5 Drive a set distance (simple command)

Drives straight forward 2 m from wherever the robot is, using odometry:

```python
def driveForwardMeters(self, distance: float) -> commands2.Command:
    start = {}

    def init():
        start["pose"] = self.drivetrain.getPose()

    def execute():
        self.drivetrain.drive(0.3, 0, 0, False)   # 30 % speed, robot-relative

    def end(interrupted):
        self.drivetrain.drive(0, 0, 0, False)

    def isFinished():
        traveled = self.drivetrain.getPose().translation().distance(
            start["pose"].translation()
        )
        return traveled >= distance

    return commands2.FunctionalCommand(init, execute, end, isFinished, self.drivetrain)

# usage
JoystickButton(self.driverController, wpilib.XboxController.Button.kA).onTrue(
    self.driveForwardMeters(2.0)
)
```

### 6.6 Telemetry to the dashboard

Add to DriveSubsystem (drivesubsystem.py):

```python
import wpilib

# in __init__
self.field = wpilib.Field2d()
wpilib.SmartDashboard.putData("Field", self.field)

# replace periodic
def periodic(self) -> None:
    self.odometry.update(self._gyroRotation(), self._modulePositions())

    pose = self.getPose()
    self.field.setRobotPose(pose)
    wpilib.SmartDashboard.putNumber("Heading deg", self.getHeading())

    for name, m in (("FL", self.frontLeft), ("FR", self.frontRight),
                    ("RL", self.rearLeft), ("RR", self.rearRight)):
        s = m.getState()
        wpilib.SmartDashboard.putNumber(f"{name} speed m/s", s.speed)
        wpilib.SmartDashboard.putNumber(f"{name} angle deg", s.angle.degrees())
        wpilib.SmartDashboard.putNumber(f"{name} target m/s", m.desiredState.speed)
        wpilib.SmartDashboard.putNumber(
            f"{name} drive A", m.drivingSpark.getOutputCurrent()
        )
```

Compare "speed" to "target" while tuning drive gains.

### 6.7 Robot-relative chassis speeds (needed by most path planners)

Add to DriveSubsystem:

```python
from wpimath.kinematics import ChassisSpeeds

def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        (
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.rearLeft.getState(),
            self.rearRight.getState(),
        )
    )

def driveRobotRelative(self, speeds: ChassisSpeeds) -> None:
    states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds)
    self.setModuleStates(states)
```

With getPose(), resetOdometry(), getRobotRelativeSpeeds() and driveRobotRelative(), the subsystem has everything a path-following library expects.

### 6.8 Your own trajectory auto

```python
from wpimath.geometry import Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

config = TrajectoryConfig(2.0, 2.0)                       # m/s, m/s^2
config.setKinematics(DriveConstants.kDriveKinematics)

traj = TrajectoryGenerator.generateTrajectory(
    Pose2d(0, 0, Rotation2d(0)),
    [Translation2d(1.5, 0.5)],                            # waypoints
    Pose2d(3, 1, Rotation2d.fromDegrees(90)),             # end pose
    config,
)
```

Build the SwerveControllerCommand exactly as getAutonomousCommand() does, passing traj instead of exampleTrajectory, and call resetOdometry(traj.initialPose()) before it runs.

### 6.9 Auto chooser on the dashboard

```python
# in __init__
self.chooser = wpilib.SendableChooser()
self.chooser.setDefaultOption("S-curve example", self.getExampleAuto)
self.chooser.addOption("Drive 2 m", lambda: self.driveForwardMeters(2.0))
self.chooser.addOption("Do nothing", lambda: commands2.InstantCommand())
wpilib.SmartDashboard.putData("Auto", self.chooser)

def getAutonomousCommand(self) -> commands2.Command:
    return self.chooser.getSelected()()
```

Rename the existing getAutonomousCommand() to getExampleAuto() first. The chooser stores functions so each auto is built fresh when autonomous starts.

### 6.10 Zero heading for the opposite alliance

If drivers stand on the red side and want "forward" to mean away from them:

```python
def zeroHeadingForAlliance(self) -> None:
    self.drivetrain.zeroHeading()
    alliance = wpilib.DriverStation.getAlliance()
    if alliance == wpilib.DriverStation.Alliance.kRed:
        self.drivetrain.gyro.setAngleAdjustment(180)
    else:
        self.drivetrain.gyro.setAngleAdjustment(0)
```

### 6.11 Reading a single module directly

```python
fl = self.drivetrain.frontLeft
print(fl.turningEncoder.getPosition())          # raw absolute angle, rad
print(fl.drivingEncoder.getVelocity())          # m/s
print(fl.turningSpark.getMotorTemperature())    # deg C
```

---

## 7. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| python -m robotpy sync fails on rev or navx | listed under components | they belong in requires (pyproject.toml above) |
| One wheel points the wrong way at rest | encoder not zeroed, or zeroed with key not seated | redo 4.1 for that module |
| Robot won't drive; "Drive held still: steering encoders not read yet" | a roboRIO steering encoder has no signal | check its 3 wires and DIO channel; Drive/XX absolute deg reads -1 for the dead one |
| "Steering encoder XX ... reads the same value while the wheel turned" | that encoder's reading is frozen (magnet loose, signal wire shorted to a steady level) | fix the encoder; until then that module steers on the NEO sensor only and is not corrected |
| Wrong wheel moves on the dashboard when you turn one by hand | DIO channels swapped | fix the k...SteeringEncoderDio values |
| Drive/XX resyncs keeps climbing while driving | steering backlash or a slipping steering gear | inspect that module; a few per match is normal |
| All wheels off by the same 45 or 90 degrees | angular offsets changed, or modules bolted in a non-standard orientation | restore constants; if mounting differs, adjust that module's ChassisAngularOffset by the rotation |
| One wheel spins backward | DrivingMotorOnBottom wrong for that module | flip it |
| A wheel twitches or spins continuously while steering | TurningMotorOnBottom wrong (motor fights the encoder) | flip it |
| Field-relative drives sideways or rotates the wrong way | gyro direction | keep getRotation2d(); check navX com type; let it calibrate |
| Robot drifts in heading over a match | navX moved during calibration, or mounted on a flexing plate | stiffen the mount; zero heading (Start) before the match |
| Odometry distance short or long | wheel diameter worn | measure a real wheel, set kWheelDiameterMeters (also check kDrivingMotorReduction is 6.3) |
| Spirals when driving and turning together | wrong kTrackWidth / kWheelBase | measure wheel centers again |
| Wheels buzz at rest | turn P too high | lower it, or add small D |
| Settings lost after brownout | not persisted | the template persists on every startup; make sure the robot code ran once after any SPARK reset |
| SPARK config errors at startup | old SPARK firmware | update to v26.1.0+ in Hardware Client 2 |

---

## 8. Simulation and automated tests

physics.py makes the simulated robot move. RobotPy loads it automatically for sim and test; the real robot ignores it.

### 8.1 What the model does

Each 20 ms loop is split into 1 ms steps, like the SPARK's own loop. In each step it:

1. works out each SPARK's output voltage: the open-loop voltage autotune asked for, or the SPARK's closed loop (P and feedforward with the gains currently applied)
2. moves each steering and drive motor with a DC motor model (volts = kS x sign(speed) + kV x speed + kA x acceleration) using "true" constants
3. writes the result into the simulated encoders, with the drive speed reading lagging like a real SPARK's

Then it moves the robot on the sim field and turns the simulated navX.

Because the SPARK loops are simulated, gains matter in the sim: bad gains overshoot or lag here too, and autotune has a real motor to measure. It does not model motor inversion, encoder zeroing, V1/V2 timing, wheel slip, battery sag or current limits (section 4.6 covers those on the real robot).

### 8.2 Running the GUI sim

```bash
python -m robotpy sim
```

1. In the sim GUI, drag a connected Xbox controller (or keyboard 0) from System Joysticks onto Joystick slot 0.
2. Click Teleoperated. Drive.
3. Open the Field widget (NetworkTables > SmartDashboard > Field) to see the robot move.

### 8.3 Terminal output

While enabled, physics.py prints a status line every 0.5 s:

```
t=  12.40 teleop truth x=  1.84 y=  0.02 h=    0.3 | odo x=  1.84 y=  0.02 h=    0.3 err=0.001m | FL    0.2deg  2.95m/s FR    0.2deg  2.95m/s RL    0.2deg  2.95m/s RR    0.2deg  2.95m/s
```

| Field | Meaning |
|---|---|
| t | sim time, s |
| auto / teleop | robot mode |
| truth x y h | where the simulated robot really is (m, m, degrees) |
| odo x y h | where the robot code's odometry thinks it is |
| err | distance between truth and odometry, m. Should stay near 0; growth means odometry math is wrong |
| FL FR RL RR | each module's angle relative to the robot (degrees) and wheel speed (m/s) |

Other messages:

| Message | Meaning |
|---|---|
| [physics] EasySwerve sim ready: encoder scale ... | startup; scale 1.0 means the SPARK sims take radians and meters directly |
| [physics] navX simulated through sim device '...' | heading goes through the real navX object |
| [physics] navX sim device not found ... | heading is fed straight into DriveSubsystem instead; driving still works |
| [physics] odometry reset detected ... | an auto (or your code) called resetOdometry; the sim robot is placed there too (camera corrections never move the sim robot) |

Settings at the top of physics.py:

| Setting | Default | Use |
|---|---|---|
| PRINT_PERIOD_S | 0.5 | status line period; 0 turns it off |
| INITIAL_STEER_ANGLE_RAD | 0.0 | start the wheels crooked to check they straighten on enable |
| SIM_WHEEL_SLIP | 0.0 | fraction of wheel travel that doesn't move the robot; set 0.05 to watch odometry drift (and the vision add-on correct it) |
| SIM_STEER_KS, KV, KA | 0.15, 0.25, 0.006 | true steering motor constants (V, rad) |
| SIM_DRIVE_KS, KV, KA | 0.12, 2.45, 0.35 | true drive motor constants (V, m) |
| SIM_DRIVE_SPEED_LAG_S | about 0.026 | how far the simulated drive speed reading lags; follows the speed measurement settings in constants.py |
| SUBSTEPS | 20 | 1 ms physics steps per loop |

Running autotune in the sim should find the SIM_ constants; change them to see how it responds.

To save output for sharing:

```bash
python -m robotpy sim 2>&1 | tee sim-log.txt
```

### 8.4 Automated tests

```bash
python -m robotpy test
```

Runs RobotPy's built-in tests plus tests/test_drive_sim.py:

| Test | Checks |
|---|---|
| test_forward_stick_drives_forward | left stick forward moves +X, no drift or rotation |
| test_left_stick_left_strafes_left | left stick left moves +Y |
| test_right_stick_left_turns_counterclockwise | right stick left spins counterclockwise in place |
| test_field_relative_forward_after_turning | after turning about 90 degrees, forward still goes down the field |
| test_x_lock_points_wheels_in_x | right bumper makes the X, wheels stopped |
| test_odometry_tracks_a_square | a 4-leg square (about 1.2 m legs) closes within 0.15 m |
| test_encoder_version_presets | V1 and V2 presets both load |
| test_bad_encoder_version_rejected | an encoder version other than 1 or 2 raises ValueError |
| test_fit_recovers_known_constants | the motor model fit finds known constants (steering-like and drive-like) |
| test_fit_with_encoder_quantization | still accurate with the NEO's coarse built-in encoder |
| test_fit_rejects_backward_motor | a backward motor fails with an inversion message instead of tuning |
| test_fit_rejects_motor_that_never_moved | a dead motor fails |
| test_delay_estimate | speed-reading lag is measured correctly |
| test_gain_rules_are_sane | gain rules give sensible values and move the right way |
| test_step_response_metrics | overshoot and settle time are measured correctly |
| test_full_autotune_in_sim | full autotune in the sim finds the simulated motor's constants and saves them |
| test_zero_steering_encoders | dashboard zeroing is refused while enabled, saved and applied while disabled |
| test_startup_sets_neo_sensor_to_true_angle | roboRIO wiring: the NEO steering sensor is set to the absolute angle at startup |
| test_drift_corrected_when_wheel_still | 2 degrees of drift is corrected once the wheel is still |
| test_small_disagreement_left_alone_while_enabled | under 1 degree is left alone while driving |
| test_no_correction_while_wheel_moving | no corrections while the wheel is steering |
| test_disabled_resync_is_rate_limited | while disabled, re-syncs at most twice a second |
| test_unplugged_encoder_never_syncs_and_warns | an unplugged encoder blocks driving and warns |
| test_stuck_encoder_is_ignored | a frozen encoder is detected and ignored, and trusted again once it changes |
| test_wraparound_takes_short_way | corrections across 0/360 degrees take the short way |
| test_set_offset_resyncs | new zeros take effect immediately |

Save the results to share:

```bash
python -m robotpy test 2>&1 | tee test-log.txt
```

A failing test prints the assert message with the measured pose, which points at the cause.

---

## 9. Autotune

Autotune measures the real motors and sets every drivetrain gain. No hand tuning.

### 9.1 What it tunes

| Gain | Used for | How it is found |
|---|---|---|
| steer P | steering angle loop | motor model fit, then real 60 degree steps adjusted until overshoot is 3 to 10 % |
| steer kS (friction push) | holding the exact angle against friction | 80 % of the measured steering friction |
| drive kS | volts to overcome friction | motor model fit |
| drive kV | volts per m/s | motor model fit |
| drive P | correcting speed error | from the fit's kA and the measured speed-reading lag (SIMC rule) |

Results are applied to the SPARKs immediately, saved to a file, and loaded at every startup. A tuning file saved before steerKS existed still loads; steerKS just stays 0 until you rerun steering autotune. On the roboRIO the file is /home/lvuser/easyswerve-tuning.json, outside the deploy folder, so redeploying code keeps your tuning. In the sim it is easyswerve-tuning.json in the project folder.

### 9.2 Running it

1. Battery charged (above 12.5 V), robot on carpet.
2. Steering: the robot stays in place; wheels spin on their pivots. About 30 s.
3. Drive: the robot drives forward up to 2.0 m, then back to where it started, four times. Clear at least 2.5 m in front of it. About 25 s.
4. Driver Station: select Test mode, then Enable.
5. Dashboard (Shuffleboard, Elastic or Glass), under Autotune/, click one of:

| Button | Does |
|---|---|
| 1 Steering (robot stays put) | steering only |
| 2 Drive (needs clear floor ahead) | drive only (wheels are pointed straight first) |
| All | steering, then drive |
| Reset to defaults | forget saved tuning, back to REV's gains |

6. Watch Autotune/Status, or the console. Disable at any time to stop instantly; nothing is saved from a stopped run.

Run steering first (or All): the drive test uses the steering loop to hold the wheels straight.

### 9.3 The report

The console prints a report you can copy. Example from the simulator:

```
[autotune] ==== EasySwerve autotune: steering + drive ====
[autotune] starting gains: drive P=0.0400 kS=0.000 V kV=2.504 V/(m/s) | steer P=1.000 kS=0.000 V (REV template defaults)
[autotune] steering model fits (volts, rad):
[autotune]   FL: kS=0.1500 kV=0.2500 kA=0.00587 R2=1.0000 rows=842
[autotune]   ...
[autotune]   median kS=0.1500 kV=0.2500 kA=0.00587 -> starting P=0.394 (damping ratio 0.75), friction push 0.120 V
[autotune] steering check-and-adjust:
[autotune]   step 1: P=0.394  worst overshoot 1.6% FL  settle 0.10 s
[autotune]   step 2: P=0.512  worst overshoot 5.1% FL  settle 0.14 s
[autotune]   steering P = 0.512, friction push = 0.120 V (worst holding error about 0.28 deg; 1.40 deg without the push)
[autotune] drive model fits (volts, m):
[autotune]   FL: kS=0.1200 kV=2.4500 kA=0.34877 R2=1.0000 rows=456
[autotune]   ...
[autotune]   median kS=0.1200 kV=2.4500 kA=0.34877; speed reading lag 25 ms
[autotune]   drive P = 0.4650 (SIMC rule), feedforward kS=0.120 V, kV=2.450 V/(m/s)
[autotune] final gains:    drive P=0.4650 kS=0.120 V kV=2.450 V/(m/s) | steer P=0.512 kS=0.120 V (autotune)
[autotune] saved to /home/lvuser/easyswerve-tuning.json (loaded automatically at every startup)
```

How to read it:

| Item | Meaning | Look for |
|---|---|---|
| kS | volts just to start moving (friction) | similar on all 4 modules |
| kV | volts per unit of speed | drive near 2.50 (the gearing predicts this); similar on all 4 |
| kA | volts per unit of acceleration | drive kA grows with robot mass |
| R2 | how well the model fits, 1.0 is perfect | above 0.95 is good; below 0.9 fails |
| speed reading lag | how far behind the SPARK's speed reading is | about 15 to 40 ms with the template's measurement settings; over 100 ms means they were not applied |
| worst holding error | how close steering can hold an angle against friction | under 0.5 deg is good |
| step lines | each steering check, with the worst module | ends with overshoot between 3 and 10 % |
| WARNING lines | a module far from the others, or kV far from the gearing | fix the hardware and rerun |

### 9.4 Safety and failure messages

Autotune only runs in Test mode, stops instantly when disabled, and the drive test stops at the distance limit. If anything fails, the previous gains stay in place and nothing is saved.

| Message | Meaning |
|---|---|
| switch the Driver Station to Test mode and enable first | started outside Test mode |
| stopped (robot disabled or command interrupted) | you disabled it; rerun when ready |
| fit failed for FR: model not physical ... check motor inversion | that motor moves opposite to its voltage: wrong MotorOnBottom setting or wiring |
| fit failed ... not enough moving data | a motor barely moved: unplugged, breaker, CAN ID, or seized |
| fit failed ... poor fit | very noisy data: binding, slipping belt, loose wiring |
| module RL steered only 12 of 60 deg | that steering motor or encoder is not working |
| wheels did not line up straight | encoder zeroing or steering problem; run steering autotune first |
| WARNING: RL kS=0.450 is far from the others | that module is binding or rubbing |
| WARNING: kV=... but the gearing predicts ... | wheel diameter, gearing or a drive motor problem |

### 9.5 Settings (constants.py, AutotuneConstants)

| Setting | Default | Use |
|---|---|---|
| kRequireTestMode | True | refuse to run outside Test mode |
| kDriveMaxDistanceM | 2.0 | farthest the drive test goes in each direction; lower it for small spaces (1.0 still works) |
| kDriveSmoothing | 1.5 | drive speed loop: 1.0 fastest (about 8 % overshoot), 2.0 gentlest (about 1 %) |
| kSteerDampingRatio | 0.75 | starting point for steering P before the real steps adjust it |
| kSteerFrictionFraction | 0.8 | steering friction push as a fraction of measured friction; below 1.0 so the push alone never moves the wheel |
| kSteerOvershootMin, kSteerOvershootMax | 0.03, 0.10 | the steering overshoot band autotune aims for |
| kSteerVerifyStepDegrees | 60 | size of the steering check steps |
| kSteerVerifyMaxIterations | 6 | most steering check steps |
| kDriveRampVoltsPerSecond, kDriveStepVolts | 0.75, 5.0 | drive test voltages |
| kSteerRampVoltsPerSecond, kSteerStepVolts | 0.5, 3.0 | steering test voltages |
| kRestSeconds | 1.0 | pause between tests |
| kOutlierFraction | 0.30 | how different a module must be to get a WARNING |

### 9.6 How it works

1. Open-loop tests: slow voltage ramps (to measure friction and speed per volt) and sudden steps (to measure acceleration), both directions, recording position every 20 ms.
2. Model fit (tuning/fit.py): the same motor model as WPILib's SysId, volts = kS x sign(speed) + kV x speed + kA x acceleration, fitted per module with an instrumental-variable least squares. Plain least squares would be badly biased by the NEO's coarse encoder; the instrumental-variable version is not, and the tests check this.
3. Steering P: start from the model for damping ratio 0.75, then real steps on all 4 wheels. Over 10 % overshoot lowers P, under 3 % raises it. Steering is too fast to model precisely from 20 ms samples, so the real steps decide the final value.
4. Steering friction push: 80 % of the fitted steering kS, sent in the direction of the error whenever the wheel is more than 0.25 degrees off. It cancels most of the friction, so a small error still gets enough voltage to move, cutting the holding error several times. DriveSubsystem re-aims it every loop.
5. Drive: feedforward kS and kV come straight from the fit. P comes from the SIMC tuning rule for a motor with a measurement delay: P = kA / ((smoothing + 1) x lag), converted to SPARK units.
6. All four modules are fitted separately; the median is used, and any module far from the others is reported.
7. All gains assume voltage compensation at kNominalVoltage, so they hold on a fresh, tired or sagging battery.

### 9.7 API

| Call | What it does |
|---|---|
| Autotuner(drive) | create (RobotContainer does this) |
| steeringCommand(), driveCommand(), fullCommand() | the commands behind the dashboard buttons |
| resetCommand() | delete saved tuning, apply defaults |
| failed, lines | after a run: whether it failed, and the report lines |
| tuning.gains.load(), save(g), reset(), path() | the saved tuning file |
| tuning.gains.TunedGains | driveP, driveKS, driveKV, steerP, steerKS plus source, tunedAt, notes |
| DriveSubsystem.applyGains(g) | send gains to all 8 SPARKs right away |
| DriveSubsystem.modules | (name, module) pairs FL, FR, RL, RR |
| EasySwerveModule.setDrivingVoltage(v), setTurningVoltage(v) | open-loop voltage (autotune only) |
| EasySwerveModule.holdAngle(angle) | steer to an angle without the 180 degree flip |
| EasySwerveModule.applyGains(g) | send gains to that module's 2 SPARKs |

Environment variable EASYSWERVE_TUNING_FILE overrides where the tuning file lives (the tests use it).

### 9.8 Battery voltage

Gains do not need to change with battery level. The motor constants autotune measures are in volts, and the SPARKs run with voltage compensation (kNominalVoltage, 12 V): they continuously rescale their output from the battery voltage they measure, about 1000 times a second, so the same gains behave the same on a fresh battery, a tired one, or one sagging under load mid-match. If the battery sags below 12 V the motors simply reach full output a little sooner.

What does change a robot's behavior over time is motor temperature, tread wear, carpet and robot mass. Rerun autotune after mechanical changes (adding a mechanism, new treads).

### 9.9 Measured effect (simulator)

| | Before | After |
|---|---|---|
| Drive speed reading lag | 140 ms | 25 ms |
| Drive P chosen by autotune | 0.083 | 0.47 |
| Drive speed step, time to 90 % | 0.14 s | 0.06 s |
| Drive speed overshoot | 6.0 % | 1.5 % |
| Steering holding error on small moves (worst) | 1.22 deg | 0.27 deg |

"Before" is autotune with REV's default speed measurement and no friction push; "After" adds the faster speed measurement and the friction push. Both use the simulated motor in physics.py; real numbers will differ, but the direction of the improvement is the same.
