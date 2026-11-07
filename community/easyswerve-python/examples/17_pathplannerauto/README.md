# 17: Run an auto from the PathPlanner app

## What it does

Run an auto that was drawn in the PathPlanner app, by its name.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot at one end of the practice carpet, facing the long way, centered side to side, with the back bumper about 20 cm from the edge, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The paths are 1.2 m long, so they fit the carpet; on a bigger floor, drag their points out in the PathPlanner app.

## The code to look at

```python
self.autos = Autos(self.drivetrain)                         # autos drawn in the PathPlanner app
```

```python
return self.autos.auto("Drive Forward")                     # The name as it appears in the PathPlanner app
```

- **Autos(self.drivetrain)** connects PathPlanner to the drivetrain.
- **self.autos.auto("Drive Forward")** builds the auto saved as deploy/pathplanner/autos/Drive Forward.auto.

## Key ideas

- **Drawing instead of coding.** PathPlanner lets you draw smooth curved paths on a picture of the field. The robot follows them using its position sensors.
- **Paths and autos.** A path is one drawn line. An auto is a list of steps: paths, waits, commands.
- **Autos start by setting the position.** "Reset Odom" (on by default) tells the robot it's at the start of the first path. On the real robot, you place it there.
- **Field coordinates.** Positions are meters from the blue alliance corner of the field.

## Try this

1. Install the PathPlanner app, open this project folder in it, and look at the Drive Forward auto and its path.
2. Drag the end of the Drive Forward path somewhere else, save, and run this example again.
3. Change the name to "Test S-Curve" to run the other auto.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
