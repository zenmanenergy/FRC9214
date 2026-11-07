# 19: Drive a path, then turn around

## What it does

Mix a PathPlanner path with a simple move in one sequence.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot at one end of the practice carpet, facing the long way, centered side to side, with the back bumper about 20 cm from the edge, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The paths are 1.2 m long, so they fit the carpet; on a bigger floor, drag their points out in the PathPlanner app.

## The code to look at

```python
return commands2.SequentialCommandGroup(                    # these run one after another:
    self.autos.drivePath("Drive Forward"),                  # 1. drive a path from the PathPlanner app
    self.drivetrain.turnTo(180),                            # 2. turn to face 180 degrees (backwards)
)
```

- A path command and a turnTo command sit in the same SequentialCommandGroup like any other commands.

## Key ideas

- **Everything is a command.** PathPlanner paths, simple moves, waits and prints all mix freely. You can build an auto in code from any of them.

## Try this

1. Add a WaitCommand between the path and the turn.
2. Add driveDistance(1.0) after the turn, to drive back toward the start.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
