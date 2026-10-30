# 15: Drive, then lock the wheels

## What it does

Drive to a spot, then lock the wheels in an X so other robots can't push yours away.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**.

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
2. Watch the wheel angles in the status line while locked: 45 and -45 degrees (a wheel may show 135 or -135 instead: the same line, rolling the other way).

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
