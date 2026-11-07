# 11: Back and forth, again and again

## What it does

Repeat a short back-and-forth move over and over for 8 seconds.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py).

## The code to look at

```python
seconds = carpet.fitSeconds(1.0, forward=0.25)              # 1 s, or less to stay on the carpet
backAndForth = commands2.SequentialCommandGroup(            # one trip, saved in a variable:
    self.drivetrain.driveFor(seconds, forward=0.25),        # forward for 1 s...
    self.drivetrain.driveFor(seconds, forward=-0.25),       # ...then back for 1 s
)
return backAndForth.repeatedly().withTimeout(8.0)           # repeat the trip, stop after 8 s
```

- **backAndForth** is a variable holding a command. Giving it a name makes the last line easier to read.
- **.repeatedly()** makes a command that starts it again every time it finishes.
- **.withTimeout(8.0)** makes a command that stops it after 8 seconds, even in the middle of a move.

## Key ideas

- **Decorators** like repeatedly and withTimeout wrap a command to change how it runs. They chain: one after another.
- **Stopping in the middle is safe.** When a command is stopped early, it still runs its cleanup, so driveFor stops the wheels.

## Try this

1. Change 8.0 to 3.0. Where is the robot when it stops?
2. Remove .withTimeout(8.0). Now it repeats until you click Disabled.
3. Use withTimeout on its own: driveFor(10.0, forward=0.25).withTimeout(1.0). Which time wins?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
