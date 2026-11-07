# 09: Wait, then drive

## What it does

Do nothing for 3 seconds, then drive. In a match this keeps your robot out of an alliance partner's way.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py). Nothing happens for 3 seconds, then it drives.

## The code to look at

```python
seconds = carpet.fitSeconds(2.0, forward=0.25)              # 2 s, or less to stay on the carpet
return commands2.SequentialCommandGroup(                    # these run one after another:
    commands2.WaitCommand(3.0),                             # 1. do nothing for 3 seconds
    self.drivetrain.driveFor(seconds, forward=0.25),        # 2. then drive forward for 2 seconds
)
```

- **WaitCommand(3.0)** is a command that does nothing and finishes after 3 seconds. In a sequence, it delays everything after it.

## Key ideas

- **Waiting is a command too.** Anything that takes time can be a step in a sequence.

## Try this

1. Change the wait to 1 second.
2. Copy the square from 08 here and put a short wait between each move.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
