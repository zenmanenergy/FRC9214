# 07: Drive in an arc

## What it does

Drive forward and turn at the same time to make a curve, like a car going around a corner.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot on the floor with about 3 m of clear space around it and someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**.

## The code to look at

```python
return self.drivetrain.driveFor(3.0, forward=0.3, turn=0.15)
```

- **forward and turn together.** Because forward means "the way the robot faces" and the robot keeps turning, the path bends into a circle.

## Key ideas

- **Turning radius = speed / turning rate.** Here: speed 0.3 x 4.5 = 1.35 m/s, turning 0.15 x 360 = 54 degrees per second = 0.94 radians per second. Radius = 1.35 / 0.94 = about 1.4 m.
- **Compare with 06.** In 06 the robot kept facing forward, so it went in a straight line. Turning while driving forward is what makes the curve.

## Try this

1. Make the curve tighter: bigger turn, or smaller forward. Predict the radius first.
2. How long would it take to drive a full circle? (360 / 54 degrees per second.) Try that time.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
