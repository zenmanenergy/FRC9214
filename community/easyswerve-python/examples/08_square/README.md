# 08: Drive a square

## What it does

Four moves, one after another: forward, left, back, right. The robot ends about where it started (time-based moves are never exact, so it may be some centimeters off).

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**.

## The code to look at

```python
return commands2.SequentialCommandGroup(
    self.drivetrain.driveFor(1.5, forward=0.25),
    self.drivetrain.driveFor(1.5, left=0.25),
    self.drivetrain.driveFor(1.5, forward=-0.25),
    self.drivetrain.driveFor(1.5, left=-0.25),
)
```

- **SequentialCommandGroup** runs commands in order: the next one starts when the one before finishes.
- The whole group is one command, so makeAuto can return it like any other.
- The robot never turns: it slides left and back. A swerve robot doesn't need to turn to drive a square.

## Key ideas

- **Command groups** build big behaviors out of small ones. Almost every real auto is a sequence.
- **Each command finishes on its own** (driveFor after its time), which is what lets the next one start.

## Try this

1. Make a rectangle: change the times on two of the moves.
2. Add a fifth move to the end (remember the comma after the line before it).
3. A real robot won't end exactly where it started. Why not? (Wheels slip, speeding up and slowing down take time.)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
