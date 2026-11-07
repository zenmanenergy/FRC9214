# 35: Drive up to an AprilTag

## What it does

Line up in front of an AprilTag automatically. Hold A and the robot drives itself to a spot 1 m in front of a tag, facing it, steering around field obstacles. The tag's spot comes from the field map; the robot's position comes from wheels, gyro and cameras together. Good for lining up to score or pick up.

## On the robot

**Setup first (once):** set VISION_ENABLED = True in robot_settings.py, remove the # in front of photonlibpy in pyproject.toml, and run py -m robotpy sync before deploying. The camera names in robot_settings.py (CAMERAS) must match the names in the PhotonVision web page exactly, and photonlibpy's version must match the PhotonVision version on the Orange Pi (docs/VISION.md). With vision off the example still runs; it just prints that vision is off.

This needs measured cameras (measured=True; docs/VISION.md). Copy this folder's robot.py over robot.py and deploy. Put the robot about 3 m from the tag, a little to the side, where the cameras see tags, with a clear path and someone ready to disable. Enable in **Autonomous**: it drives slowly to 1 m in front of the tag. The console prints the goal position at startup.

## The code to look at

```python
self.goal = self.vision.poseInFrontOfTag(TAG_ID, STOP_DISTANCE_M)    # the spot in front of the tag
```

```python
x, y = self.goal.X(), self.goal.Y()
facing = self.goal.rotation().degrees()
return self.autos.driveTo(x, y, facing, maxSpeedMps=1.5)             # slowly (1.5 m/s at most)
```

```python
self.controller.a().whileTrue(self.driveToGoal())                # hold A: drive there by itself
```

- **poseInFrontOfTag(TAG_ID, 1.0)** looks the tag up in the field map and works out the spot 1 m in front of it, facing it.
- **autos.driveTo(x, y, heading)** uses PathPlanner to plan a path from wherever the robot is to that spot, and drive it.
- **whileTrue**: hold A to go, let go to stop and get control back.
- **maxSpeedMps=1.5** keeps it slow while you're testing.

## Key ideas

- **This needs measured cameras.** Driving to a spot only works if the robot knows where it is. Unmeasured cameras (measured=False) don't correct the position, so the robot goes where it thinks the spot is, using wheels only.
- **Field map.** Every tag's position is published by FIRST each season. The code loads it, so you name a tag number instead of measuring the field.
- **Pathfinding.** The robot plans around obstacles in the PathPlanner navgrid (deploy/pathplanner/navgrid.json).
- **Tuned, not guessed.** While it drives, PathPlanner's PID compares where the robot is with where the path says it should be and corrects. Its P, I and D are the position and heading gains from setup step 11 (00_11_autotuneposition). PathPlanner reads them when the robot code starts, so restart the code (or deploy) after tuning.

## Try this

1. Change TAG_ID to another tag number. Where does it drive?
2. Change STOP_DISTANCE_M to 0.5. Does it stop closer?
3. Add a second button: B drives to a different tag.
4. Real robot: measure your cameras first (docs/VISION.md), set measured=True, then try it slowly on a practice field.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
