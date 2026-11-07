# 03: Drive backward

## What it does

The same as 02, but backward. A negative speed means the other direction.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py).

## The code to look at

```python
seconds = carpet.fitSeconds(2.0, forward=-0.25)             # 2 s, or less to stay on the carpet
return self.drivetrain.driveFor(seconds, forward=-0.25)     # negative forward = backward, 2 seconds
```

- **forward=-0.25**: the minus sign reverses the direction. There's no separate "backward" command.

## Key ideas

- **Signs carry direction.** Positive forward, negative backward. The same idea comes back for left/right and for turning.

## Try this

1. What do you think forward=0 does? Try it.
2. Measure the distance with a tape measure and compare with example 02. Same distance, the other way?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
