# 07: Drive in an arc

## What it does

Drive forward and turn at the same time to make a curve, like a car going around a corner.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py).

## The code to look at

```python
seconds = carpet.fitSeconds(3.0, forward=0.3, turn=0.15)          # 3 s, or less to stay on the carpet
return self.drivetrain.driveFor(seconds, forward=0.3, turn=0.15)  # drive and turn at once: a curve
```

- **forward and turn together.** Because forward means "the way the robot faces" and the robot keeps turning, the path bends into a circle.

## Key ideas

- **Turning radius = speed / turning rate.** Here: speed 0.3 x 1.5 = 0.45 m/s, turning 0.15 x 180 = 27 degrees per second = 0.47 radians per second. Radius = 0.45 / 0.47 = about 0.96 m.
- **Compare with 06.** In 06 the robot kept facing forward, so it went in a straight line. Turning while driving forward is what makes the curve.

## Try this

1. Make the curve tighter: bigger turn, or smaller forward. Predict the radius first.
2. How long would it take to drive a full circle? (360 / 54 degrees per second.) Try that time.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
