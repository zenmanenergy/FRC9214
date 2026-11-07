# 14: Drive an L

## What it does

Drive, turn a corner, drive again. Both moves use sensors, so the corner is a true 90 degrees.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py).

## The code to look at

```python
first = carpet.fitMeters(1.5)                               # 1.5 m, or less: the long way
second = carpet.fitMeters(1.5, carpet.SIDE_M)               # after the turn it goes across: less room
return commands2.SequentialCommandGroup(                    # these run one after another:
    self.drivetrain.driveDistance(first),                   # drive that far the way the robot faces
    self.drivetrain.turnTo(90),                             # then turn to face 90 degrees (left)
    self.drivetrain.driveDistance(second),                  # drive that far the way the robot faces
)
```

- After turnTo(90), the robot faces left. driveDistance drives in the direction the robot faces, so the second leg goes left on the field.

## Key ideas

- **Combining sensor moves.** driveDistance (12) plus turnTo (13) can drive any shape made of straight lines.
- **Compare with the square in 08.** There the robot slid without turning. Here it turns like a car. Both work on swerve; which is better depends on where the robot needs to face.

## Try this

1. Make a full square this way. Remember turnTo is absolute: the corners are turnTo(90), turnTo(180), turnTo(-90), then turnTo(0).
2. Make a triangle (hint: 120 degree corners).

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
