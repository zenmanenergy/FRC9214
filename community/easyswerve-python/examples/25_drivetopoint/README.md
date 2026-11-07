# 25: Drive to a spot by itself

## What it does

Hold A and the robot drives itself to a spot on the field, steering around obstacles. Let go and the driver is back in control.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Plug an Xbox controller into the Driver Station laptop (it should be in slot 0 on the USB tab), clear some space, and enable in **Teleoperated**. This one needs a big floor or a field: it drives to a spot on the field, not on the carpet. Drive somewhere, then hold A.

## The code to look at

```python
goTo = self.autos.driveTo(2.0, 4.0, 0)                      # a command: drive to x=2, y=4, facing 0
self.controller.a().whileTrue(goTo)                         # hold A: drive there by itself
```

- **driveTo(2.0, 4.0, 0)**: go to 2 m by 4 m from the blue alliance corner, facing 0 degrees (away from the blue wall).
- **whileTrue**: the trip only continues while A is held. Letting go cancels it.

## Key ideas

- **Pathfinding.** PathPlanner plans a route from wherever the robot is, around the obstacles in deploy/pathplanner/navgrid.json.
- **Planned at the moment you press.** The route depends on where the robot is right then.
- **This needs to know where the robot is on the field.** Wheels and gyro alone drift a little; cameras (docs/VISION.md) keep it accurate on a real field.
- **Tuned, not guessed.** While it drives, PathPlanner's PID compares where the robot is with where the path says it should be and corrects. Its P, I and D are the position and heading gains from setup step 11 (00_11_autotuneposition). PathPlanner reads them when the robot code starts, so restart the code (or deploy) after tuning.

## Try this

1. Change the target to another spot. The field is about 16.5 m long and 8 m wide.
2. Add a second button that drives to a different spot.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
