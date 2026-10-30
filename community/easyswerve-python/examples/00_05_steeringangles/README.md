# Setup step 5: Point all four wheels the same way

## What it does

The "do the wheels turn in sync, to the right angle" test. Now the steering uses its PID: every wheel is told to point the same way (straight, left, right, diagonal, backwards) and the console shows how close each one got and how long it took to settle. All four should end up parallel, within a couple of degrees, at about the same time.

## On the robot

1. Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy).
2. Robot on blocks, wheels off the floor.
3. Enable in **Teleoperated** and press the D-pad: up = straight ahead, left = 90 left, right = 90 right, down = backwards. Or enable in **Autonomous** to go through them all.
4. LOOK: are all four wheels parallel, pointing the way the console says?
5. A wheel off or never settling: go back to step 3 (directions) and step 4 (zeros).

## The code to look at

```python
for name, module in self.drivetrain.modules:
    self.distance[name] = abs(angleError(degrees, module.angleRelativeToRobotDeg()))
    module.pointAt(degrees)
```

```python
error = angleError(degrees, module.angleRelativeToRobotDeg())
if abs(error) > CLOSE_DEG:
    self.lastOff[name] = now - self.startTime
```

```python
speeds = {n: self.distance[n] / t for n, t in self.arrived.items() if t > 0}
```

- **pointAt(degrees)** steers one wheel to an angle relative to the robot (0 = straight ahead, 90 = left) with the PID, and keeps the drive motor still.
- **angleRelativeToRobotDeg()** is where the code THINKS the wheel points. If the zeros are wrong, this can be perfect while the real wheel points somewhere else. That's why you also look.
- **Settled** means it got within 2 degrees and stayed there. The code remembers the last time each wheel was outside 2 degrees.
- **Turning speed** (degrees moved divided by time) is fair to compare between wheels; plain time isn't, because some wheels have farther to turn.

## Key ideas

- **Code belief vs. reality.** Sensors only measure what they measure. Zeroing is what connects the encoder's numbers to the real wheel.
- **In sync** means each wheel responds the same way to the same command. A wheel that settles much slower is often binding, has a loose belt, or a different gear ratio.
- **Bevel gear side.** EasySwerve wheels zeroed with the bevel gear on the wrong side point exactly backwards. They'd still drive, but backwards, which looks like a backwards drive motor.

## Try this

1. Press up, then left, a few times. Do the wheels settle at the same time every time?
2. Add a fifth direction to makeAuto, like self.pointAll(-45).
3. Change CLOSE_DEG to 0.5. Does every wheel still settle? (The NEO sensor sees about 0.4 degree steps.)

Next: [00_06_driveonebyone](../00_06_driveonebyone/README.md)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
