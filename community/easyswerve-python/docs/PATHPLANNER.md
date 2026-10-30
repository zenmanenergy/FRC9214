# EasySwerve PathPlanner - Reference and Guide

> **Coach reference.** This guide was written before the project was simplified, so some file names below are old. Where things are now:
>
> | Old name in this guide | Now |
> |---|---|
> | constants.py: CAN IDs, DIO ports, motor mounting, sizes, encoder location and version, gyro | robot_settings.py |
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


A small add-on library that connects PathPlanner to the EasySwerve RobotPy drivetrain template.

- Separate from the drivetrain: it lives in its own folder and imports nothing from it.
- No drivetrain changes needed: it only calls methods the template already has (getPose, resetOdometry, setModuleStates, and each module's getState).
- Built for EasySwerve: it knows the module's wheel size, 6.3:1 gearing, NEO / NEO 2.0 motor and current limit, so the robot config is one function call.

Units are meters, seconds, radians (degrees only where a function name says so).

---

## Contents

1. How the pieces fit
2. Install
3. PathPlanner app setup
4. Wiring it into robotcontainer.py
5. API reference
6. Examples
7. Tuning
8. Troubleshooting
9. Trying it in the simulator

---

## 1. How the pieces fit

```
PathPlanner app (laptop)        draws paths and autos, saves them into
                                deploy/pathplanner/ in your robot project

robotpy-pathplannerlib          PathPlanner's own robot library (reads those
                                files, follows paths)

easyswerve_pathplanner          this library: connects the two above to the
                                EasySwerve drivetrain

EasySwerve drivetrain template  drives the four modules
```

Files:

```
easyswerve_pathplanner/
  __init__.py       exports EasySwervePathPlanner, robotconfig, hardware
  planner.py        EasySwervePathPlanner class
  robotconfig.py    load or build the PathPlanner RobotConfig
  hardware.py       EasySwerve module facts (wheel, gearing, motor)
examples/
  robotcontainer.py the template's robotcontainer.py with PathPlanner added
  robot.py          the template's robot.py, keeping the odometry field widget current
  deploy/pathplanner/
    settings.json   robot config for an EasySwerve robot (edit mass and size)
    navgrid.json    open field for on-the-fly driving (replace with this season's)
    paths/          7 starter paths (section 3.1)
    autos/          6 starter autos (section 3.1)
```

---

## 2. Install

1. Copy the easyswerve_pathplanner folder into your robot project, next to robot.py:

```
your-robot/
  robot.py
  robotcontainer.py
  constants.py
  configs.py
  subsystems/
  easyswerve_pathplanner/     <- here
  deploy/pathplanner/         <- the PathPlanner app creates this
  pyproject.toml
```

2. Add PathPlanner's library to pyproject.toml:

```toml
requires = [
    "robotpy-rev==2026.0.4",
    "robotpy-navx==2026.0.1.2",
    "robotpy-pathplannerlib==2026.1.2",
]
```

3. Copy examples/deploy/pathplanner/ into your project's deploy/ folder (so you have deploy/pathplanner/settings.json and so on), and copy examples/robotcontainer.py and examples/robot.py over the template's two files. Your existing autotune and calibration buttons are kept in them.

4. Run:

```bash
python -m robotpy sync
```

---

## 3. PathPlanner app setup

1. Install the PathPlanner app (2026 version) on the laptop.
2. Open your robot project folder in it. It creates deploy/pathplanner/.
3. In app Settings, switch units to metric if they aren't.
4. Settings > Robot Config, fill in for EasySwerve:

| Field | Value |
|---|---|
| Holonomic mode | on |
| Robot mass | your robot with bumpers and battery, kg |
| Robot MOI | start with mass x (length^2 + width^2) / 12, kg m^2 |
| Wheel radius | 0.0508 m |
| Drive gearing | 6.3 |
| True max drive speed | 4.5 m/s to start; lower it to what the robot really reaches on carpet |
| Wheel COF | 1.2 (1.0 if unsure) |
| Drive motor | NEO (use this for NEO 2.0 too) |
| Drive current limit | 50 A |
| Module offsets | FL (+wheelBase/2, +trackWidth/2), FR (+, -), BL (-, +), BR (-, -). For the template default 0.6858 m square: 0.3429 m |
| Bumper size | your bumper outside dimensions |

5. Draw paths and autos. Draw everything from the blue alliance side; the library mirrors them for red.

If you'd rather keep robot facts in code, skip step 4 and use robotconfig.fromValues() (section 5.2).

The starter settings.json already has the EasySwerve values filled in, with placeholders for mass (52 kg), bumper size (0.89 m) and MOI. Open it in the app and correct those three.

### 3.1 Starter paths and autos

All positions are field meters from the blue corner. Every path is drawn from the blue side and mirrored on red. Each auto resets odometry to its first path's start, so place the robot there (or anywhere in the simulator).

Test paths (folder Tests), for checking and tuning the drive:

| Path | What it does | Checks |
|---|---|---|
| Test Straight 2m | (2.0, 4.0) to (4.0, 4.0), facing forward, max 2 m/s | translation P, odometry distance |
| Test Strafe 1.5m | (2.0, 4.0) to (2.0, 5.5) sideways, heading held at 0 | sideways tracking, heading hold |
| Test Spin While Driving | (2.0, 4.0) to (5.0, 4.0) while turning 0 to 180 degrees | rotation P while translating |
| Test S-Curve | (2.0, 3.0) through (3.5, 4.2) to (5.0, 3.0); faces 90 degrees at the middle | curves, rotation targets |

Example paths (folder Examples), showing PathPlanner features:

| Path | Shows |
|---|---|
| Example Out | event marker "Say hello" at the halfway point runs named command printHello; constraint zone "Slow near the end" limits the last 30 % to 1 m/s |
| Example Back | the return trip, robot facing forward while driving backward |
| Score Approach | a short final approach, facing the wall (180 degrees), max 1.5 m/s; used by teleop auto-align (example 6.3) |

Autos (one per test path, plus two examples):

| Auto | Sequence |
|---|---|
| Test Straight, Test Strafe, Test Spin While Driving, Test S-Curve | that one path |
| Example Out and Back | Example Out, xLock (0.5 s), wait 0.5 s, printHello, Example Back |
| Example Intake While Driving | Example Out with intakeDemo running alongside (deadline group: intake stops when the path ends), wait 0.5 s, Example Back |

They use the named commands printHello, xLock and intakeDemo, which examples/robotcontainer.py registers.

---

## 4. Wiring it into robotcontainer.py

The full working version is examples/robotcontainer.py. The order matters:

```python
from easyswerve_pathplanner import EasySwervePathPlanner

class RobotContainer:
    def __init__(self):
        self.drivetrain = DriveSubsystem()

        # 1. named commands first
        EasySwervePathPlanner.registerNamedCommand("intake", IntakeCommand(...))

        # 2. connect PathPlanner to the drivetrain (once)
        self.planner = EasySwervePathPlanner(
            self.drivetrain, DriveConstants.kDriveKinematics
        )

        # 3. auto chooser
        self.autoChooser = self.planner.buildAutoChooser()

    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()()
```

You can delete the template's example S-curve auto (the old getAutonomousCommand body) once this works. PathPlanner autos reset the odometry to their own starting pose, so you don't call resetOdometry yourself.

---

## 5. API reference

### 5.1 EasySwervePathPlanner (planner.py)

```python
EasySwervePathPlanner(
    drive,                      # the EasySwerve DriveSubsystem
    kinematics,                 # DriveConstants.kDriveKinematics
    *,
    robotConfig=None,           # None = load from PathPlanner app settings
    translationPID=PIDConstants(5.0, 0.0, 0.0),
    rotationPID=PIDConstants(5.0, 0.0, 0.0),
    flipForRedAlliance=True,
)
```

Create exactly one. Construction calls PathPlanner's AutoBuilder.configure, which can only be done once per robot program.

| Method | Returns | What it does |
|---|---|---|
| registerNamedCommand(name, command) | None | static. Makes a command usable by name in the app's autos and event markers. Call before building autos. |
| buildAuto(autoName) | Command | a full auto from the app, by file name without .auto |
| followPath(pathName) | Command | one path from the app, by file name without .path |
| pathfindToPose(pose, constraints, goalEndVelocityMps=0.0) | Command | drive on the fly to a field pose (blue-origin coordinates), avoiding obstacles in the app's navgrid |
| pathfindThenFollowPath(pathName, constraints) | Command | pathfind to the start of a saved path, then follow it exactly |
| autoNames() | list of str | every auto saved in the project, sorted; empty list if none |
| buildAutoChooser(defaultAutoName="", dashboardKey="Auto") | SendableChooser | dashboard chooser of all autos plus "Do nothing". Holds builder functions: use chooser.getSelected()() |
| logToField(field) | None | static. Draws the active path and target pose on a wpilib.Field2d |
| getRobotRelativeSpeeds() | ChassisSpeeds | measured speeds from the 4 modules (PathPlanner calls this) |
| driveRobotRelative(speeds, feedforwards=None) | None | drives at robot-relative speeds (PathPlanner calls this). Feedforwards are ignored because the SPARKs already apply kV |
| shouldFlipPath() | bool | True on red alliance when flipForRedAlliance is on |
| startAt(pathName) | Command | resets odometry to a saved path's starting pose (mirrored on red); first step of an auto built in code |

Attributes: drive, kinematics, robotConfig, flipForRedAlliance.

Module-level constants: DEFAULT_TRANSLATION_PID, DEFAULT_ROTATION_PID.

### 5.2 robotconfig (robotconfig.py)

| Function | Returns | What it does |
|---|---|---|
| fromApp() | RobotConfig | loads deploy/pathplanner/settings.json; raises FileNotFoundError with instructions if missing |
| fromValues(massKg, moiKgM2, trackWidthM, wheelBaseM, wheelCOF=1.2, maxDriveSpeedMps=None) | RobotConfig | builds the config in code with EasySwerve wheel, gearing, motor and 50 A limit |
| boxMOI(massKg, lengthM, widthM) | float | starting moment of inertia estimate for a box-shaped robot |
| settingsFilePath() | str | where the app's settings file should be |

### 5.3 hardware (hardware.py)

| Name | Value |
|---|---|
| WHEEL_DIAMETER_M | 0.1016 |
| WHEEL_RADIUS_M | 0.0508 |
| DRIVE_REDUCTION | 6.3 |
| NEO_FREE_SPEED_RPM | 5676 |
| WHEEL_FREE_SPEED_MPS | about 4.79 |
| DRIVE_CURRENT_LIMIT_A | 50 |
| driveMotor() | DCMotor.NEO(1) with 6.3 reduction |

---

## 6. Examples

### 6.1 Named commands used inside an auto

In the app, add a Named Command block called "intake" to an auto, or an event marker on a path. In code:

```python
EasySwervePathPlanner.registerNamedCommand(
    "intake", commands2.StartEndCommand(intake.run, intake.stop, intake).withTimeout(1.5)
)
```

### 6.2 Run one path from a button (testing)

```python
JoystickButton(self.driverController, wpilib.XboxController.Button.kY).whileTrue(
    self.planner.followPath("Test Straight 2m")
)
```

Hold Y to run it, release to stop. The path is field-relative, so start the robot near its beginning, (2.0, 4.0).

### 6.3 Teleop auto-align to a scoring spot

```python
from pathplannerlib.path import PathConstraints

constraints = PathConstraints(3.0, 3.0, 3.14, 6.28)   # m/s, m/s^2, rad/s, rad/s^2

JoystickButton(self.driverController, wpilib.XboxController.Button.kB).whileTrue(
    self.planner.pathfindThenFollowPath("Score Approach", constraints)
)
```

Held = drives there; released = sticks take over.

### 6.4 Drive to a pose chosen at the moment the button is pressed

```python
JoystickButton(self.driverController, wpilib.XboxController.Button.kA).whileTrue(
    commands2.DeferredCommand(
        lambda: self.planner.pathfindToPose(self.pickTarget(), constraints),
        self.drivetrain,
    )
)
```

DeferredCommand builds the path when pressed, so pickTarget() can use the latest pose or vision data.

### 6.5 Robot config in code

```python
from easyswerve_pathplanner import robotconfig

cfg = robotconfig.fromValues(
    massKg=52.0,
    moiKgM2=robotconfig.boxMOI(52.0, 0.85, 0.85),
    trackWidthM=DriveConstants.kTrackWidth,
    wheelBaseM=DriveConstants.kWheelBase,
    maxDriveSpeedMps=3.8,
)
self.planner = EasySwervePathPlanner(
    self.drivetrain, DriveConstants.kDriveKinematics, robotConfig=cfg
)
```

### 6.6 Never mirror paths (practice field, symmetric game)

```python
self.planner = EasySwervePathPlanner(
    self.drivetrain, DriveConstants.kDriveKinematics, flipForRedAlliance=False
)
```

### 6.7 Show the path on the dashboard

```python
field = wpilib.Field2d()
wpilib.SmartDashboard.putData("Field", field)
EasySwervePathPlanner.logToField(field)
```

Also call field.setRobotPose(self.drivetrain.getPose()) in a periodic so the robot appears next to the path.

---

### 6.8 Event markers: run a command partway along a path

In the app, add an event marker to a path, set its position along the path, and give it a named command. The starter "Example Out" path does this: its marker at 0.5 (halfway between the two waypoints) runs printHello. In code you only register the name (example 6.1). The marker fires while the robot keeps driving.

For a command that should run over a stretch of path, set the marker's end position too (a zone): it starts at the first position and is stopped at the second.

### 6.9 Run a mechanism during a path (deadline group)

The "Example Intake While Driving" auto puts the path and the named command intakeDemo in a deadline group. The first command is the deadline: when the path ends, intakeDemo is stopped. In the app: add a Deadline Group to the auto, put the path first and the named command second.

```python
EasySwervePathPlanner.registerNamedCommand(
    "intakeDemo",
    commands2.StartEndCommand(intake.run, intake.stop, intake),  # runs until stopped
)
```

### 6.10 Slow down for part of a path (constraint zone)

The "Example Out" path has a constraint zone "Slow near the end" from 0.7 to 1.0 that limits speed to 1 m/s. Use zones for tight spots or for the last approach to a scoring spot. Nothing is needed in code.

### 6.11 Face a direction partway (rotation target)

The "Test S-Curve" path has a rotation target at 1.0 (the middle waypoint) of 90 degrees: the robot turns to face 90 degrees by then, and back to 0 by the end. A holonomic robot can point anywhere while it drives the path; rotation targets and the goal end rotation say where.

### 6.12 Build an auto in code instead of the app

Useful for autos that pick paths based on sensors, or to keep autos in version control as code.

```python
def codeAuto(self) -> commands2.Command:
    p = self.planner
    return commands2.SequentialCommandGroup(
        p.startAt("Example Out"),                    # like the app's Reset Odom
        p.followPath("Example Out"),
        commands2.RunCommand(self.drivetrain.setX, self.drivetrain).withTimeout(0.5),
        commands2.WaitCommand(0.5),
        p.followPath("Example Back"),
    )

# put it in the chooser next to the app's autos
self.autoChooser.addOption("Code: out and back", self.codeAuto)
```

### 6.13 Pick one of several scoring spots with the D-pad

```python
from commands2.button import Trigger

SPOTS = {
    0: Pose2d(1.6, 6.5, Rotation2d.fromDegrees(180)),    # D-pad up
    90: Pose2d(1.6, 5.0, Rotation2d.fromDegrees(180)),   # D-pad right
    270: Pose2d(1.6, 3.5, Rotation2d.fromDegrees(180)),  # D-pad left
}
for angle, pose in SPOTS.items():
    Trigger(lambda a=angle: self.driverController.getPOV() == a).whileTrue(
        commands2.DeferredCommand(
            lambda p=pose: self.planner.pathfindToPose(p, TELEOP_PATH_CONSTRAINTS),
            self.drivetrain,
        )
    )
```

Held = drives there, released = sticks back. The a=angle and p=pose defaults make each button keep its own spot.

### 6.14 Let the driver take over by moving the sticks

Auto-drive commands require the drive subsystem, so any other command that requires it interrupts them. This stops auto-drive the moment a stick is pushed, even if the button is still held:

```python
from commands2.button import Trigger

def sticksMoved() -> bool:
    c = self.driverController
    return max(abs(c.getLeftX()), abs(c.getLeftY()), abs(c.getRightX())) > 0.3

Trigger(sticksMoved).onTrue(
    commands2.InstantCommand(lambda: None, self.drivetrain)  # takes the drive, ending auto-drive
)
```

The default drive command takes over again right after.

### 6.15 Auto-align only when close enough

Pathfinding from across the field takes time; limit auto-align to when the robot is already near the target:

```python
TARGET = Pose2d(1.6, 6.5, Rotation2d.fromDegrees(180))

def nearTarget() -> bool:
    return self.drivetrain.getPose().translation().distance(TARGET.translation()) < 3.0

JoystickButton(self.driverController, wpilib.XboxController.Button.kX).whileTrue(
    commands2.DeferredCommand(
        lambda: self.planner.pathfindToPose(TARGET, TELEOP_PATH_CONSTRAINTS),
        self.drivetrain,
    ).onlyIf(nearTarget)
)
```

### 6.16 Different autos for each starting spot

Draw one auto per starting position (left, center, right) in the app; they all appear in the chooser automatically. Name them so the drive team can't mix them up, for example "Left - 2 piece", "Center - 1 piece". Set the default with buildAutoChooser(defaultAutoName="Center - 1 piece") so a forgotten selection still does something safe.

### 6.17 Use the test paths to tune path following

1. Run the Test Straight auto. Compare the end of the path (4.0, 4.0) to where the robot stops (Odometry Field widget, or the simulator's status line).
2. Short of the end or drifting sideways: raise translationPID kP. Weaving: lower it.
3. Run Test Spin While Driving. Heading lags the plan: raise rotationPID kP. Heading wobbles: lower it.
4. Run Test S-Curve to check both together on a curve.

Do this after drivetrain autotune (the drivetrain must already hold its commanded speeds).

## 7. Tuning

Do the drivetrain tuning in the EasySwerve guide first; path following can't fix a drive that doesn't hold its commanded speed.

1. Set the true max drive speed (app) or maxDriveSpeedMps (code) to what the robot actually reaches on carpet. Too high = paths it can't keep up with.
2. Draw a 3 m straight path. Run it. Compare where the robot stops to where the path ends.
3. translationPID kP (default 5): raise if the robot ends short or drifts off the line; lower if it wobbles side to side.
4. rotationPID kP (default 5): raise if heading lags; lower if it oscillates.
5. MOI too low makes rotation planning aggressive and the robot overshoots heading; raise it.

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| FileNotFoundError about settings.json at startup | app robot config never saved into this project | open the project in the app and save Robot Config, or use robotconfig.fromValues() |
| "AutoBuilder has already been configured" | EasySwervePathPlanner created twice | create one, in RobotContainer |
| Chooser only shows "Do nothing" | no autos saved, or app pointed at another folder | check deploy/pathplanner/autos/ exists in this project |
| Named command block does nothing | registered after the auto was built | register before EasySwervePathPlanner / buildAutoChooser |
| Robot drives the path mirrored on blue | flipping wrong | paths must be drawn from the blue side |
| Auto starts in the wrong place on the field | odometry not reset | autos reset pose from their starting pose; make sure "Reset Odom" is on in the auto's settings |
| Teleop field-relative is rotated 180 degrees after a red auto | template's field-relative drive uses the raw navX heading, not the pose | see the note below |
| Path followed but overshoots corners | true max speed or acceleration set higher than the robot can do | lower them in the app |

Note on field-relative after autos: the drivetrain template turns stick input into field directions using the raw navX heading, measured from wherever the robot faced at power-on or the last Start press. A PathPlanner auto resets the odometry pose, but not the navX, so after an auto that started at an angle, teleop "forward" can be off by that angle.

The fix is to drive field-relative from the odometry pose instead, which PathPlanner keeps correct in blue-origin field coordinates. A red driver faces the opposite way, so add 180 degrees on red. In the template's DriveSubsystem.drive(), replace the fieldRelative branch with:

```python
if fieldRelative:
    heading = self.getPose().rotation()
    if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
        heading = heading + Rotation2d.fromDegrees(180)
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeedDelivered, ySpeedDelivered, rotDelivered, heading
    )
```

(add import wpilib at the top of drivesubsystem.py). With this change, the Start button's zeroHeading() no longer affects driving direction. To re-align mid-match, point the robot straight away from your own driver station and reset only the pose heading: Rotation2d() on blue, Rotation2d.fromDegrees(180) on red, for example resetOdometry(Pose2d(getPose().translation(), Rotation2d())) on blue.

---

## 9. Trying it in the simulator

Everything above runs in the RobotPy simulator, with the drivetrain's physics.py moving the robot. Good for learning PathPlanner before the robot is ready.

1. Set up as in section 2, including the starter deploy/pathplanner files and the two example files.
2. Start the simulator:

```bash
python -m robotpy sim
```

3. In the sim GUI: open the Field widget (true position) and Odometry Field (where the code thinks it is, plus the planned path in another color).
4. Pick an auto in the Auto chooser on the dashboard, then click Autonomous.
5. Watch the console. physics.py prints a status line every 0.5 s:

```
t=   3.40 auto   truth x=  3.12 y=  4.00 h=    0.0 | odo x=  3.12 y=  4.00 h=    0.0 err=0.002m | FL    0.1deg  1.95m/s ...
```

Lines that start with [auto] are the named commands running (printHello, intakeDemo on and off).

6. For teleop features: plug in an Xbox controller, drag it to Joystick 0, click Teleoperated, hold A, B or Y.

What to look for:

| Check | Good |
|---|---|
| err in the status line | stays near 0 (odometry matches the true position) |
| robot ends at the path's end point | within a few cm |
| [auto] hello from PathPlanner | printed about halfway along Example Out |
| [auto] intake on / off | on at the start of Example Out, off when it ends |

To save the output to share:

```bash
python -m robotpy sim 2>&1 | tee pathplanner-sim.txt
```
