# 06: Drive diagonally

## What it does

Combine forward and sideways to drive at an angle, while still facing straight ahead.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**.

## The code to look at

```python
return self.drivetrain.driveFor(2.0, forward=0.2, left=0.2)
```

- **forward=0.2 and left=0.2 together** add up like arrows (vectors): equal parts forward and left point 45 degrees to the left.
- The robot keeps facing forward the whole time. Only the direction of travel is diagonal.

## Key ideas

- **Vectors.** Speed has a size and a direction. Forward and left are the two parts. The combined speed is sqrt(0.2 x 0.2 + 0.2 x 0.2) = 0.28.

## Try this

1. Try forward=0.3, left=0.1. The angle of travel is atan(0.1 / 0.3), about 18 degrees. Check it with the truth x and y at the end.
2. Try forward=0.2, left=-0.2. Which way does it go?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
