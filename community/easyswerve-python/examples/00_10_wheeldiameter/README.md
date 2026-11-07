# Setup step 10: Measure the wheel diameter

## What it does

Make the robot's distances match the real world. The drive encoders count wheel turns; the code turns that into meters using WHEEL_DIAMETER_M in robot_settings.py. Treads wear down, so the real diameter shrinks and every distance (and every PathPlanner path) comes out wrong. This drives a test distance, you measure how far it really went, and the robot calculates the right diameter.

## On the robot

1. Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy).
2. On the practice carpet (its size is in robot_settings.py): put the robot at one end, facing the long way, centered side to side, with the back bumper about 20 cm from the edge. Put a tape mark on the floor at the front bumper.
3. Enable in **Autonomous**. The robot drives 2 m slowly (less if the carpet is too short) and stops. The console says how close to that distance it stopped, by its encoders: within a centimeter or two is good.
4. Measure from the tape mark to the front bumper and type it into the dashboard box "Wheel Check/Measured distance (m)", then disable.
5. Copy the suggested WHEEL_DIAMETER_M into robot_settings.py and deploy. Run it again to check.

## The code to look at

```python
pairs = zip(self.startDistances, end)                              # (start, end) for each wheel
rolled = [abs(e - s) for s, e in pairs]                            # how far each wheel rolled
self.encoderDistance = sum(rolled) / len(rolled)                   # the average of the 4
```

```python
ratio = measured / self.encoderDistance                            # real distance / encoder distance
correct = robot_settings.WHEEL_DIAMETER_M * ratio                  # the real diameter: same ratio
```

- **wheelDistances()** gives how far each wheel has rolled, by its encoder. The difference between the start and the end, averaged over the 4 wheels, is the encoder distance.
- **The correction is one ratio.** The robot thinks it drove encoderDistance; it really drove measured. The diameter is wrong by the same ratio, so correct diameter = current diameter x measured / encoder distance.
- **speed=0.15** is slow on purpose: if the wheels slip, the encoders count turns that didn't move the robot, and the calibration would be wrong.
- **getNumber** reads a value typed into the dashboard. putNumber in robotInit creates the box.

## Key ideas

- **Why it's a ratio:** each wheel turn rolls pi x diameter. Distance = turns x pi x diameter. The turns are the same in both, so distance and diameter change in the same proportion.
- **Calibrating against a trusted measurement.** A tape measure is the reference; the code is adjusted to agree with it. This is how most sensors are calibrated.
- **One number fixes everything.** Change WHEEL_DIAMETER_M, redeploy, and driving, autos and paths are all corrected.

## Try this

1. On the real robot: put tape at the front bumper, run the auto, measure from the tape to the bumper, type it in. Copy the suggested value into robot_settings.py and redeploy. Run the test again: does it come out right now?
2. Work it out by hand first: if the robot thinks it drove 1.50 m but really drove 1.46 m, what's the new diameter?
3. Do it again at speed 0.5. Is the answer different? Why? (Hint: slipping.)
4. Repeat every few events: treads wear during competition.

Next: [00_11_autotuneposition](../00_11_autotuneposition/README.md)

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
