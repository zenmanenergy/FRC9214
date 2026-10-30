# 14: Drive an L

## What it does

Drive, turn a corner, drive again. Both moves use sensors, so the corner is a true 90 degrees.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**.

## The code to look at

```python
return commands2.SequentialCommandGroup(
    self.drivetrain.driveDistance(1.5),
    self.drivetrain.turnTo(90),
    self.drivetrain.driveDistance(1.5),
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
