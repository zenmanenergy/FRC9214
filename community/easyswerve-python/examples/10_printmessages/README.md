# 10: Print messages

## What it does

Print messages in the console while an auto runs, to see which step it's on.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py). Watch the console (Driver Station, or the VS Code terminal).

## The code to look at

```python
seconds = carpet.fitSeconds(1.5, forward=0.25)              # 1.5 s, or less to stay on the carpet
return commands2.SequentialCommandGroup(                    # these run one after another:
    commands2.PrintCommand("Auto started"),                 # print a message (shows in the console)
    commands2.WaitCommand(1.0),                             # wait 1 second
    commands2.PrintCommand("Driving forward"),              # say what's next
    self.drivetrain.driveFor(seconds, forward=0.25),        # drive forward for 1.5 s
    commands2.PrintCommand("Auto finished"),                # the last step
)
```

- **PrintCommand** prints a message and finishes right away.
- The messages come out in order, with the drive in between.

## Key ideas

- **Debugging.** When an auto does something unexpected, printing at each step shows how far it got.
- **Not every command moves something.** Commands can print, wait, change settings, anything.

## Try this

1. Copy the square from 08 and print a message before each move.
2. Print a message that includes a number, like "Driving 1.5 seconds".

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
