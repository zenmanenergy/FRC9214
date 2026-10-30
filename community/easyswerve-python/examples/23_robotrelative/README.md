# 23: Robot-relative driving

## What it does

Drive like a remote-control car: pushing forward moves the way the robot's front points.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Plug an Xbox controller into the Driver Station laptop (it should be in slot 0 on the USB tab), clear some space, and enable in **Teleoperated**. Turn the robot, then push forward.

## The code to look at

```python
self.drivetrain.driveWithController(self.controller, fieldRelative=False)
```

- **fieldRelative=False** switches off field-relative driving. Everything else is the same as example 01.

## Key ideas

- **Field-relative (the default):** pushing forward always drives away from the driver, whichever way the robot faces. The gyro makes this possible. Most drivers find it much easier.
- **Robot-relative:** forward is the robot's front. Easy to understand, hard to drive once the robot has turned around.

## Try this

1. Turn the robot 180 degrees and push forward. Then try the same in example 01. Which is easier?
2. Drive a figure eight both ways.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
