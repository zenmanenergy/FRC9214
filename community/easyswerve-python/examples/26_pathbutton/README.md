# 26: Drive a path from a button

## What it does

Hold A: the robot drives itself to the start of a path, then follows it exactly. Good for lining up the same way every time.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, plug in an Xbox controller, drag it from System Joysticks onto Joystick slot 0, and click **Teleoperated**. Hold A.

## The code to look at

```python
self.controller.a().whileTrue(self.autos.goToPath("Drive Forward 2m"))
```

- **goToPath** combines 25 and 18: pathfind to the path's start, then follow the path.

## Key ideas

- **Why not just driveTo the end?** The path controls the last part of the approach: the angle and speed the robot comes in at. That's what makes lining up repeatable, for example to score.

## Try this

1. Change the path to "S-Curve".
2. In the PathPlanner app, draw a short new path and use it here.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
