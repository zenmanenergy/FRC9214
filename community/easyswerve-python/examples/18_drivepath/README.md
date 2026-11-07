# 18: Drive a path

## What it does

Drive one path from the PathPlanner app directly, without making an auto for it.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot at one end of the practice carpet, facing the long way, centered side to side, with the back bumper about 20 cm from the edge, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The paths are 1.2 m long, so they fit the carpet; on a bigger floor, drag their points out in the PathPlanner app.

## The code to look at

```python
return self.autos.drivePath("S-Curve")                      # drive the path called S-Curve (PathPlanner app)
```

- **drivePath("S-Curve")** sets the robot's position to the start of the S-Curve path, then follows it.

## Key ideas

- **Following a path** means the robot constantly compares where it is with where the path says it should be, and corrects. It's the same measure-compare-act idea as driveDistance in 12, in two dimensions plus rotation.
- **Position reset.** The robot can only follow a path drawn on the field if it knows where it is on the field. drivePath tells it: you're at the start.

## Try this

1. Change the path to "Drive Forward".
2. In the PathPlanner app, change the S-Curve path's end rotation so the robot ends facing a different way. Run it again.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
