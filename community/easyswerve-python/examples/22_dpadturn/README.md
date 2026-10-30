# 22: Point the robot with the D-pad

## What it does

Use the D-pad (the plus-shaped pad) to turn the robot to face up, left, down or right.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Plug an Xbox controller into the Driver Station laptop (it should be in slot 0 on the USB tab), clear some space, and enable in **Teleoperated**. Press the D-pad.

## The code to look at

```python
self.controller.povUp().onTrue(self.drivetrain.turnTo(0))
self.controller.povLeft().onTrue(self.drivetrain.turnTo(90))
```

- **povUp, povLeft, povDown, povRight** are the D-pad directions. They work like buttons.
- Up means "face away from the driver" (0 degrees), left 90, down 180, right -90.

## Key ideas

- **POV** stands for point of view. The D-pad reports an angle, and each direction is a button.
- **Matching the D-pad to field directions** makes it easy for the driver to aim the robot quickly.

## Try this

1. Add the diagonals: povUpLeft is turnTo(45), povUpRight is turnTo(-45).
2. The Start button (resetHeading) isn't set up in this example. Copy that line in from 01 and see how it changes what "up" means.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
