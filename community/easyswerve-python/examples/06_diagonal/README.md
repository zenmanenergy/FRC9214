# 06: Drive diagonally

## What it does

Combine forward and sideways to drive at an angle, while still facing straight ahead.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py).

## The code to look at

```python
seconds = carpet.fitSeconds(2.0, forward=0.2, left=0.2)          # 2 s, or less to stay on the carpet
return self.drivetrain.driveFor(seconds, forward=0.2, left=0.2)  # forward AND left at once: diagonal
```

- **forward=0.2 and left=0.2 together** add up like arrows (vectors): equal parts forward and left point 45 degrees to the left.
- The robot keeps facing forward the whole time. Only the direction of travel is diagonal.

## Key ideas

- **Vectors.** Speed has a size and a direction. Forward and left are the two parts. The combined speed is sqrt(0.2 x 0.2 + 0.2 x 0.2) = 0.28.

## Try this

1. Try forward=0.3, left=0.1. The angle of travel is atan(0.1 / 0.3), about 18 degrees. Measure how far forward and how far left it went, and check the angle.
2. Try forward=0.2, left=-0.2. Which way does it go?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
