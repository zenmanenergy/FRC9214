# 04: Drive sideways

## What it does

Slide sideways without turning. This is what makes swerve special: a normal (tank) drive robot can't do it.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot on the floor with about 3 m of clear space around it and someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. Watch it slide to its left without turning.

## The code to look at

```python
return self.drivetrain.driveFor(2.0, left=0.25)
```

- **left=0.25** slides left at quarter speed. Negative slides right.
- **forward and left are from the robot's point of view.** left means toward the robot's left side.

## Key ideas

- **Swerve modules steer.** All four wheels turn to point sideways, then drive. The robot doesn't have to face where it's going.
- **Three ways to move:** forward/back, left/right, and turning. A swerve robot can do all three at once.

## Try this

1. Change to left=-0.25 to slide right.
2. Watch the wheels: they all point sideways while sliding.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
