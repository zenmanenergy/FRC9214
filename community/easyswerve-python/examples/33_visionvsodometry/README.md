# 33: Wheels only vs. wheels + cameras

## What it does

See why cameras help. The robot tracks its position two ways at once: wheels only (encoders and gyro, called odometry) and wheels + cameras (the drivetrain's real position, corrected whenever a camera sees a tag). Once a second it prints both and how far apart they are. With slipping wheels, the wheels-only position drifts and the cameras pull the real one back.

## Run it

**Setup first (once):** set VISION_ENABLED = True in robot_settings.py. For the simulator, also remove the # in front of photonlibpy in pyproject.toml and run py -m robotpy sync. On the real robot, the camera names in robot_settings.py (CAMERAS) must match the names in the PhotonVision web page exactly, and photonlibpy's version must match the PhotonVision version on the Orange Pi (docs/VISION.md). With vision off the example still runs; it just prints that vision is off.

To make the wheels slip, set SIM_WHEEL_SLIP = 0.05 at the top of easyswerve/sim/physics.py. Copy this folder's robot.py over robot.py, run py -m robotpy sim, and click **Autonomous**. The robot drives back and forth in front of a tag. Compare both positions with truth x= y= in the simulator's status line, which is where the robot really is.

## The code to look at

```python
self.wheelsOnly = SwerveDrive4Odometry(
    self.drivetrain.kinematics,
    self.drivetrain.gyroRotation(),
    self.drivetrain.modulePositions(),
)
```

```python
self.wheelsOnly.update(self.drivetrain.gyroRotation(), self.drivetrain.modulePositions())
```

```python
self.wheelsOnly.resetPosition(
    self.drivetrain.gyroRotation(),
    self.drivetrain.modulePositions(),
    self.drivetrain.getPose(),
)
```

- **SwerveDrive4Odometry** is WPILib's wheels-only position tracker. It needs the robot's shape (kinematics), the gyro heading, and how far each wheel has rolled in which direction (module positions).
- **update** must run every loop (robotPeriodic) with fresh readings, or the tracker falls behind.
- **resetPosition** puts both trackers at the same starting spot, so any difference later comes from the cameras.
- **getPose()** on the drivetrain is the fused position: the same wheel tracking, plus camera corrections.

## Key ideas

- **Odometry adds up small errors.** Each loop adds a tiny move; slips and bumps add tiny mistakes that never go away, so the error grows the longer you drive.
- **Cameras don't drift, but they're noisy and not always there.** Sensor fusion (a pose estimator) mixes them: wheels for smooth fast updates, cameras to pull the error back.
- **Trust depends on distance.** Close tags with several in view count a lot; one far tag counts a little.

## Try this

1. Run it with SIM_WHEEL_SLIP = 0 and then 0.05. How far apart do the two get each time?
2. Set VISION_ENABLED = False. Now both positions should match exactly. Why?
3. Change the auto to drive back and forth longer (withTimeout(30)). Does the wheels-only error keep growing?
4. Real robot: drive around the field for a minute, then park on a line you can measure to. Which position is closer?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
