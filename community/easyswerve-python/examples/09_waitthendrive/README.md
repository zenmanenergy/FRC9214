# 09: Wait, then drive

## What it does

Do nothing for 3 seconds, then drive. In a match this keeps your robot out of an alliance partner's way.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot on the floor with about 3 m of clear space around it and someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. Nothing happens for 3 seconds, then it drives.

## The code to look at

```python
return commands2.SequentialCommandGroup(
    commands2.WaitCommand(3.0),
    self.drivetrain.driveFor(2.0, forward=0.25),
)
```

- **WaitCommand(3.0)** is a command that does nothing and finishes after 3 seconds. In a sequence, it delays everything after it.

## Key ideas

- **Waiting is a command too.** Anything that takes time can be a step in a sequence.

## Try this

1. Change the wait to 1 second.
2. Copy the square from 08 here and put a short wait between each move.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
