# 03: Drive backward

## What it does

The same as 02, but backward. A negative speed means the other direction.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot on the floor with about 3 m of clear space around it and someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**.

## The code to look at

```python
return self.drivetrain.driveFor(2.0, forward=-0.25)
```

- **forward=-0.25**: the minus sign reverses the direction. There's no separate "backward" command.

## Key ideas

- **Signs carry direction.** Positive forward, negative backward. The same idea comes back for left/right and for turning.

## Try this

1. What do you think forward=0 does? Try it.
2. Measure the distance with a tape measure and compare with example 02. Same distance, the other way?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
