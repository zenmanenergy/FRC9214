# 04: Drive sideways

## What it does

Slide sideways without turning. This is what makes swerve special: a normal (tank) drive robot can't do it.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**. Watch the robot slide left on the Field widget.

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
2. Watch the wheel angles in the terminal status line (FL, FR, RL, RR): they all point sideways while sliding (90 or -90 degrees).

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
