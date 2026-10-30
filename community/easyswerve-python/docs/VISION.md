# EasySwerve Vision (PhotonVision add-on)

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


Uses AprilTags seen by PhotonVision cameras (on an Orange Pi) to correct the EasySwerve robot's position on the field.

The drivetrain tracks its position every 20 ms from the drive encoders, the steering encoders and the navX ("dead reckoning"). That is smooth and fast, but it drifts: wheels slip, the robot gets bumped. Cameras give an absolute position whenever they see tags, but less often and with some noise. The drivetrain's pose estimator (a Kalman filter) combines both: wheels and gyro moment to moment, cameras pulling it back to the truth, each weighted by how trustworthy it is. It also accounts for camera delay, applying each camera reading at the moment the picture was taken.

Everything that uses the robot's position (field-relative driving, PathPlanner autos and auto-align) gets the corrected position automatically. Nothing else needs changing.

All units are meters, seconds and degrees unless a setting name says radians.

---

## Contents

1. What happens when
2. Install
3. Setting up PhotonVision on the Orange Pi
4. Measuring your cameras (replacing the placeholders)
5. Checking that it works
6. Settings
7. Simulator
8. Troubleshooting
9. With PathPlanner
10. Files and API

---

## 1. What happens when

The robot always drives. Cameras only ever add information.

| Situation | Robot position comes from | Dashboard Vision/Status |
|---|---|---|
| photonlibpy not installed | wheels + gyro | photonlibpy not installed: wheels + gyro only |
| VISION_ENABLED = False | wheels + gyro | off (VISION_ENABLED = False) |
| Orange Pi off, unplugged or booting | wheels + gyro | no camera connected: wheels + gyro only |
| Pi connected, camera still has placeholder values (measured = False) | wheels + gyro (camera is "watch only": shown on the dashboard, not used) | camera connected but watch only (not measured) |
| Pi connected, camera measured | wheels + gyro + camera | wheels + gyro + camera (front) |
| A bad frame (ambiguous tag, too far, robot spinning, off the field) | that frame ignored | reason under Vision/front/last reject reason |
| Camera library error (for example a version mismatch) | wheels + gyro; error reported every 5 s at most | Vision/front/errors counts up |

The Pi can be plugged in or rebooted at any time; cameras are picked up the moment they start sending, with no robot restart.

Before a match, while the robot is disabled, a camera that sees 2 or more tags sets the robot's position directly (no blending), so the robot knows where it is on the field before auto starts.

---

## 2. Install

1. Copy the easyswerve_vision folder into your robot project, next to robot.py.

2. Add photonlibpy to requires in pyproject.toml. Its version must match the PhotonVision version on the Orange Pi (shown on PhotonVision's Settings page):

```toml
requires = [
    "robotpy-rev==2026.0.4",
    "robotpy-navx==2026.0.1.2",
    "robotpy-pathplannerlib==2026.1.2",
    "photonlibpy==2026.3.4",   # SAME version as PhotonVision on the Pi
]
```

Then run:

```bash
python -m robotpy sync
```

3. In robotcontainer.py, create it right after the drivetrain. The attribute must be named vision (the simulator looks for that name):

```python
from easyswerve_vision import VisionSubsystem

class RobotContainer:
    def __init__(self) -> None:
        self.drivetrain = DriveSubsystem()
        self.vision = VisionSubsystem(self.drivetrain)
        ...
```

That's all the code. It runs by itself every loop.

4. Copy the two files in tests/ into your robot project's tests/ folder.

Needs the EasySwerve drivetrain template from 2026-10-24 or later (the one with the pose estimator and addVisionMeasurement).

---

## 3. Setting up PhotonVision on the Orange Pi

1. Flash the PhotonVision image for your Orange Pi model (from PhotonVision's releases page) onto a microSD card, following PhotonVision's install guide for coprocessors.
2. Connect the Pi to the robot radio with Ethernet and power it from a regulated supply (not straight from the PDH).
3. Open the PhotonVision page from the laptop on the robot network: http://photonvision.local:5800
4. Settings: set your team number. A static IP (10.TE.AM.11, with your team number) makes it reliable; follow PhotonVision's networking guide.
5. Note the PhotonVision version shown on the Settings page, and set photonlibpy to the same version (section 2).
6. Cameras: give each camera a short name, for example front. This is the name that goes in config.py.
7. Calibrate each camera at the resolution you will use (Cameras page, calibration). AprilTag positions are only as good as the calibration.
8. Create a pipeline of type AprilTag on each camera:
   - tag family 36h11
   - 3D mode on
   - turn on multi-target estimation ("Do Multi-Target Estimation"), so the Pi combines all tags in view into one position
9. Point a camera at an AprilTag and check that PhotonVision outlines it.

PhotonVision's own documentation covers each step in detail; the names above match its web page, but a release may word them slightly differently.

---

## 4. Measuring your cameras (replacing the placeholders)

Open easyswerve_vision/config.py. Each camera has an entry like this; every number is a placeholder:

```python
CameraConfig(
    name="front",       # name in PhotonVision
    forwardM=0.30,      # lens is 0.30 m forward of the robot's center
    leftM=0.0,          # 0 m left of center (negative = right)
    upM=0.25,           # lens is 0.25 m above the floor
    yawDeg=0.0,         # points straight ahead
    tiltUpDeg=20.0,     # tilted 20 degrees up from level
    measured=False,     # set True after measuring
),
```

How to measure:

| Value | Measure from | To | Sign |
|---|---|---|---|
| forwardM | the robot's center (halfway between the wheels, front to back) | the camera lens | + toward the front |
| leftM | the robot's center (halfway between the wheels, side to side) | the lens | + toward the left side, - toward the right |
| upM | the floor | the lens | always + |
| yawDeg | straight ahead | where the camera points, seen from above | 0 ahead, 90 left, -90 right, 180 back |
| tiltUpDeg | level | where the camera points, seen from the side | + up, - down |
| rollDeg | upright | | leave 0 unless the camera is mounted on its side |

Tips:
- Measure to the center of the lens, with the robot on a flat floor.
- A phone level app works for tiltUpDeg.
- An error of 1 cm in the measurements is about 1 cm in the robot's position. An error of 1 degree in tilt or yaw grows with distance: about 5 cm at 3 m.

Then set measured=True and redeploy. From then on that camera's readings are used.

For more cameras, copy the entry, give it the camera's PhotonVision name, and measure it the same way.

---

## 5. Checking that it works

1. Deploy, connect the Driver Station, robot disabled.
2. Dashboard, under Vision/:

| Key | Expect |
|---|---|
| Status | wheels + gyro + camera (front) |
| front/connected | true |
| front/mode | fusing |
| front/tags seen | the number of tags in view |
| front/accepted | climbing while tags are in view |
| Measurements fused | climbing |

3. Put the robot on the field at a spot you can measure (for example, bumpers against the alliance wall at a known distance from a corner). Open Vision/Field: the robot should be drawn where it really is. Each camera's latest reading is drawn as its own marker.
4. Drive around in view of tags. The robot marker and camera markers should stay together. If a camera marker is consistently offset or rotated from the robot, its measurements in config.py are off (section 4).

Before any of this, while the camera still has placeholder values, mode shows watch only and you can already see its reading on Vision/Field. It just isn't used yet.

---

## 6. Settings

All in easyswerve_vision/config.py.

| Setting | Default | What it does |
|---|---|---|
| CAMERAS | one placeholder camera | your cameras (section 4) |
| VISION_ENABLED | True | master switch; False = wheels + gyro only |
| FIELD_LAYOUT | None | None = this season's field tag layout |
| MAX_SINGLE_TAG_AMBIGUITY | 0.2 | a single tag that could be read two ways is ignored above this |
| MAX_SINGLE_TAG_DISTANCE_M | 4.0 | single tags farther than this are ignored |
| MAX_MULTI_TAG_DISTANCE_M | 6.0 | multi-tag readings farther than this are ignored |
| MAX_SPIN_DEG_PER_S | 360 | ignore readings while spinning faster (motion blur) |
| FIELD_MARGIN_M | 0.5 | ignore positions more than this outside the field |
| MAX_HEIGHT_ERROR_M | 0.5 | ignore positions more than this above or below the floor |
| XY_COEFFICIENT | 0.02 | position trust: standard deviation = 0.02 x distance^2 / tags |
| HEADING_COEFFICIENT | 0.06 | heading trust, same form; single tags never change heading |
| MIN_XY_STD_M | 0.02 | most a reading is ever trusted |
| SINGLE_TAG_MULTIPLIER | 2.0 | single tags trusted half as much |
| SEED_WHILE_DISABLED_MIN_TAGS | 2 | while disabled, set the position directly when this many tags are seen; 0 = never |
| SIM_USE_UNMEASURED_CAMERAS | True | in the simulator, use placeholder cameras anyway (never affects the robot) |
| SIM_CAMERA_MODEL, SIM_CAMERA_FPS, SIM_CAMERA_LATENCY_MS | OV9281_800_600, 30, 35 | simulated camera |

In the drivetrain's constants.py, DriveConstants.kOdometryStdDevs (0.05, 0.05, 0.01) sets how much the wheels and gyro are trusted. Smaller numbers there make cameras correct the position more slowly.

---

## 7. Simulator

photonlibpy includes a camera simulator: simulated cameras see the simulated field's AprilTags from the robot's true position.

```bash
python -m robotpy sim
```

- Vision/Status shows fusing (placeholder cameras are allowed in the sim).
- Open Vision/Field next to the simulator's Field (true position).

To watch vision fix drift: in the drivetrain's physics.py set SIM_WHEEL_SLIP = 0.05 (the wheels report 5 % more travel than the robot makes). The err value in physics.py's status line grows as you drive with VISION_ENABLED = False, and stays small with it True while tags are in view.

Tests (copy tests/ into your robot project's tests/ folder):

```bash
python -m robotpy test
```

| Test | Checks |
|---|---|
| test_vision_filters.py (17 cases) | accept and reject rules, trust sizing, placeholder cameras never used on the robot |
| test_camera_estimates_are_fused_in_sim | robot placed 2 m in front of a tag in the sim; the camera's readings get fused |
| test_robot_drives_with_no_camera | with no cameras at all the robot still drives normally |

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Status: photonlibpy not installed | missing from pyproject.toml | add it (section 2), python -m robotpy sync, redeploy |
| Status: no camera connected | Pi off, not on the robot network, or camera name mismatch | check http://photonvision.local:5800 from the robot network; check name in config.py matches PhotonVision exactly |
| front/errors climbing, decode errors in the console | photonlibpy and PhotonVision versions differ | make them the same version |
| connected, but tags seen stays 0 | pipeline not AprilTag, wrong tag family, 3D mode off, or nothing in view | section 3, step 8 |
| tags seen but accepted stays 0 | every frame rejected | read front/last reject reason; loosen that setting if it's too strict for your cameras |
| mode says watch only | measured is still False | measure (section 4), set measured=True |
| robot position jumps when tags appear | a camera's measurements are wrong, or the camera isn't calibrated | compare camera markers to the robot on Vision/Field; re-measure; recalibrate in PhotonVision |
| camera marker offset by a fixed distance | forwardM or leftM wrong (or left/right sign swapped) | re-measure |
| camera marker rotated around the robot | yawDeg wrong | re-measure |
| position correct up close, wrong far away | tiltUpDeg wrong, or calibration poor | re-measure tilt, recalibrate |
| Status: setup failed | an error building the cameras (for example a bad FIELD_LAYOUT name) | read the console; the robot drives on wheels + gyro meanwhile |

---

## 9. With PathPlanner

Nothing to change: PathPlanner reads getPose(), which now includes the cameras. Paths and teleop auto-align use the corrected position automatically.

One choice to make: each PathPlanner auto has "Reset Odom" on by default, which sets the position to the path's start when the auto begins. With good cameras, the robot already knows where it is (it was seeded while disabled), so you can turn Reset Odom off in the app. Then a robot placed slightly off its mark still drives the paths in the right place on the field. Keep it on until the cameras are measured and trusted.

---

## 10. Files and API

```
easyswerve_vision/
  __init__.py   exports VisionSubsystem, robotToCamera, config, filters
  config.py     cameras (placeholders) and all settings
  filters.py    accept/reject rules and trust sizing (plain Python)
  vision.py     VisionSubsystem
tests/
  test_vision_filters.py
  test_vision_sim.py
```

| Call | What it does |
|---|---|
| VisionSubsystem(drive) | create, after the DriveSubsystem; name it self.vision |
| vision.status | the same text as Vision/Status |
| vision.cameras | per camera: cfg, connected, accepted, watched, rejected (counts by reason), lastPose, errors |
| vision.simulationUpdate(truePose) | called by physics.py in the simulator |
| robotToCamera(cameraConfig) | the camera's Transform3d on the robot |
| filters.judge(estimate) | accept or reject one reading, with its trust |
| filters.stdDevs(tagCount, avgDistanceM) | trust for a reading |

Drivetrain side (in the template):

| Call | What it does |
|---|---|
| DriveSubsystem.addVisionMeasurement(pose, timestampSeconds, stdDevs) | blend a camera position into the estimate |
| DriveSubsystem.getPose() | the combined position |
| DriveSubsystem.visionMeasurementCount | how many camera readings have been blended in |
