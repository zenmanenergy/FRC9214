# 15: Drive, then lock the wheels

## What it does

Drive to a spot, then lock the wheels in an X so other robots can't push yours away.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot on the floor with about 3 m of clear space around it and someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**.

## The code to look at

```python
return commands2.SequentialCommandGroup(
    self.drivetrain.driveDistance(2.0),
    self.drivetrain.lockWheels(),  # keeps holding until auto ends
)
```

- **lockWheels** points the four wheels in an X. Pushing the robot in any direction pushes some wheels sideways, which resists.
- **lockWheels never finishes on its own.** It holds until something stops it: the end of autonomous, or another command that needs the drivetrain.

## Key ideas

- **Commands that never finish** are fine at the end of a sequence. In the middle, they'd block everything after them.

## Try this

1. Put lockWheels in the middle of a sequence, before another move. What happens? Fix it with .withTimeout(1.0).
2. Look at the wheels while locked: they form an X. Try pushing the robot.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
