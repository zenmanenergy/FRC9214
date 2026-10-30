# 33: Check the gyro (IMU)

## What it does

Check the navX gyro, which tells the robot which way it faces. Field-relative driving, turnTo and every path depend on it. This checks the three things that go wrong with gyros: still calibrating, drifting while still, and turns that don't add up.

## Run it

Copy this folder's robot.py over robot.py, run py -m robotpy sim, leave it **Disabled** and watch the terminal, then click **Autonomous** for the turning test. The simulated gyro is perfect, so the real test is on the robot.

## The code to look at

```python
if self.drivetrain.gyroCalibrating():
    self.driftStartTime = None
```

```python
minutes = (now - self.driftStartTime) / 60.0
drift = heading - self.driftStartHeading
perMinute = drift / minutes if minutes > 0 else 0.0
```

```python
return commands2.SequentialCommandGroup(
    commands2.PrintCommand("Turning test: mark where the robot faces now"),
    self.drivetrain.turnTo(90),
    self.drivetrain.turnTo(180),
    self.drivetrain.turnTo(-90),
    self.drivetrain.turnTo(0),
    commands2.PrintCommand("Turning test done: is it facing the mark?"),
)
```

- **gyroCalibrating()** is True for the first several seconds after power-on, while the navX measures its own error. The robot must sit still the whole time.
- **Drift** is how much the heading changes while the robot isn't moving. The code remembers the heading when calibrating ends, then divides the change by the time: degrees per minute.
- **disabledInit** runs once each time the robot is disabled, so every disable starts a fresh drift test.
- **The turning test** turns to four directions and back to 0. If the gyro is right, the robot ends facing exactly where it started.

## Key ideas

- **The navX calibrates itself** at power-on, measuring its tiny built-in error so it can subtract it. If the robot moves during that, the error is measured wrong and the heading drifts for the whole match. That's why teams keep the robot still for about 20 seconds after turning it on.
- **A rate, not a number.** Drift is measured in degrees per minute, because a small error grows over time. A good gyro barely changes while still.
- **Testing against the real world.** The code can't tell if it's wrong about where it faces. A tape mark on the floor can.

## Try this

1. On the real robot: power on, wait until Calibrating is false, then watch the drift for a minute without touching the robot. It should barely move.
2. Turn the robot on and pick it up while it's calibrating. Then measure the drift. What changed?
3. Before the turning test, put tape on the floor along the robot's side. After the test, is it lined up? How many degrees off?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
