# 11: Back and forth, again and again

## What it does

Repeat a short back-and-forth move over and over for 8 seconds.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot on the floor with about 3 m of clear space around it and someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**.

## The code to look at

```python
backAndForth = commands2.SequentialCommandGroup(
    self.drivetrain.driveFor(1.0, forward=0.25),
    self.drivetrain.driveFor(1.0, forward=-0.25),
)
return backAndForth.repeatedly().withTimeout(8.0)
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
