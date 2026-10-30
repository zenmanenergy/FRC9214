# 32: What do the cameras see?

## What it does

Check that both cameras on the Orange Pi are working. PhotonVision runs on the Pi, finds AprilTags in each camera's picture, and sends the results to the roboRIO over the network. Once a second this prints whether each camera (front and back) is connected, which tag numbers it sees, and where the robot is getting its position from.

## Run it

**Setup first (once):** set VISION_ENABLED = True in robot_settings.py. For the simulator, also remove the # in front of photonlibpy in pyproject.toml and run py -m robotpy sync. On the real robot, the camera names in robot_settings.py (CAMERAS) must match the names in the PhotonVision web page exactly, and photonlibpy's version must match the PhotonVision version on the Orange Pi (docs/VISION.md). With vision off the example still runs; it just prints that vision is off.

Copy this folder's robot.py over robot.py, run py -m robotpy sim, and click **Autonomous**. The robot starts 2 m in front of a tag and spins slowly, so the front camera sees tags, then the back camera does. Watch the terminal. The dashboard (Elastic) shows more under Vision/: accepted, rejected, and the last reject reason for each camera.

## The code to look at

```python
print(f"Vision: {self.vision.status}")
```

```python
for name in self.vision.cameraNames():
    if not self.vision.isConnected(name):
        print(f"  {name:6} not connected")
        continue
    tags = self.vision.tagsSeen(name)
```

```python
spot = self.vision.poseInFrontOfTag(distanceM=2.0)
if spot is not None:
    self.drivetrain.resetOdometry(spot)
```

- **self.drivetrain.vision** is the camera system. The Drivetrain builds it from CAMERAS in robot_settings.py, so robot.py doesn't have to.
- **vision.status** is one sentence saying where the position comes from: wheels + gyro only, or wheels + gyro + camera.
- **isConnected(name)** is True while that camera is sending pictures. **tagsSeen(name)** lists the tag numbers in its picture right now.
- **poseInFrontOfTag** finds a spot in front of a tag using the field map. Here it's only used in the simulator, to start where the cameras can see something.

## Key ideas

- **Seeing a tag is not the same as using it.** The robot throws away bad sightings (too far, blurry, robot spinning fast). On the real robot, unmeasured cameras (measured=False) are watch only: they show tags but never move the robot's position. The simulator uses them anyway, because its cameras sit exactly where the placeholder numbers say.
- **Nothing breaks without the Pi.** If a camera is unplugged or the Pi is off, the robot keeps driving on wheels and gyro. That's on purpose.
- **Two cameras, two views.** Front and back together see tags in more directions, so the robot gets corrected more often.

## Try this

1. Real robot: unplug one camera's USB while this runs. What does the terminal say? Plug it back in.
2. Turn off the Pi. Can you still drive?
3. Hold a printed AprilTag in front of each camera. Do the tag numbers match what's printed on the paper?
4. In robot_settings.py, change one camera's name so it doesn't match PhotonVision. What happens?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
