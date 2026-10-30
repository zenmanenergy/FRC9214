# 34: Aim at an AprilTag

## What it does

Point the robot straight at an AprilTag. Hold A and the robot turns until the tag is in the middle of the front camera's picture, while the left stick still drives. This uses the camera's picture directly, not the field map, so it works even before the cameras are measured. Good for aiming a shooter.

## Run it

**Setup first (once):** set VISION_ENABLED = True in robot_settings.py. For the simulator, also remove the # in front of photonlibpy in pyproject.toml and run py -m robotpy sync. On the real robot, the camera names in robot_settings.py (CAMERAS) must match the names in the PhotonVision web page exactly, and photonlibpy's version must match the PhotonVision version on the Orange Pi (docs/VISION.md). With vision off the example still runs; it just prints that vision is off.

Copy this folder's robot.py over robot.py, run py -m robotpy sim, and click **Autonomous**. The robot starts 2.5 m from a tag, turned 20 degrees away, and turns to face it. Watch the Field widget.

## The code to look at

```python
tag = self.vision.target(CAMERA)
if tag is None:
    return 0.0  # no tag in sight: don't turn
```

```python
turn = -tag.getYaw() * AIM_GAIN
return max(-0.3, min(0.3, turn))  # limit to 30 % turning speed
```

```python
self.drivetrain.drive(forward, left, self.aimTurnSpeed(), True)
```

- **vision.target("front")** is what the front camera sees right now: the best tag, or None. Pass a tag number, like target("front", 7), to aim at one tag only.
- **getYaw()** is how many degrees the tag is right (+) or left (-) of the picture's center.
- **The minus sign:** tag to the right means turn right, and turning right (clockwise) is negative in WPILib.
- **AIM_GAIN** is a P controller: turn faster when far off, slower as it gets centered. The limit keeps it from spinning too fast.

## Key ideas

- **Feedback.** The camera sees the result of each turn, so the robot keeps correcting until it's centered. This is the same idea as PID, with the camera as the sensor.
- **Always handle None.** Tags go out of view all the time. The code stops turning instead of crashing.
- **Picture vs. map.** Aiming with yaw only needs the picture. Driving to a spot (example 38) needs the robot's position on the field.

## Try this

1. Change AIM_GAIN to 0.03. Does it aim faster? Does it start to wobble back and forth?
2. Change CAMERA to "back". Does the minus sign need to change? Predict first, then try it.
3. Aim at one tag only: target(CAMERA, 7). What happens when tag 7 is out of view?
4. Real robot: hold A and drive sideways with the left stick. Does the robot stay pointed at the tag?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
