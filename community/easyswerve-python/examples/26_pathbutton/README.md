# 26: Drive a path from a button

## What it does

Hold A: the robot drives itself to the start of a path, then follows it exactly. Good for lining up the same way every time.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Plug an Xbox controller into the Driver Station laptop (it should be in slot 0 on the USB tab), clear some space, and enable in **Teleoperated**. Hold A.

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
