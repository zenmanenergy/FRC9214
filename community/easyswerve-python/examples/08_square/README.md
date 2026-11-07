# 08: Drive a square

## What it does

Four moves, one after another: forward, left, back, right. The robot ends about where it started (time-based moves are never exact, so it may be some centimeters off).

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py).

## The code to look at

```python
ahead = carpet.fitSeconds(1.5, forward=0.25)                # 1.5 s, or less to stay on the carpet
side = carpet.fitSeconds(1.5, left=0.25)                    # the carpet is narrow: shorter
return commands2.SequentialCommandGroup(                    # these run one after another:
    self.drivetrain.driveFor(ahead, forward=0.25),          # 1. forward
    self.drivetrain.driveFor(side, left=0.25),              # 2. left
    self.drivetrain.driveFor(ahead, forward=-0.25),         # 3. backward
    self.drivetrain.driveFor(side, left=-0.25),             # 4. right (negative left), back to the start
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
