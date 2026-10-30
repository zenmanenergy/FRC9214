# 02: Drive forward

## What it does

The simplest possible auto: drive straight forward for 2 seconds at quarter speed, then stop. About 2 meters.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**.

Watch: The status line in the terminal: truth is where the simulated robot really is, odo is where the code thinks it is.

## The code to look at

```python
# driveFor(seconds, forward=speed). Speed 1.0 = full, 0.25 = quarter speed.
return self.drivetrain.driveFor(2.0, forward=0.25)
```

- **driveFor** is a command: drive at a fixed speed for some seconds, then stop.
- **2.0** is the time in seconds.
- **forward=0.25** is the speed as a fraction of top speed. Top speed is MAX_SPEED_MPS in robot_settings.py (4.5 m/s), so 0.25 is about 1.1 m/s.
- **makeAuto returns the command**, and autonomousInit starts it. You don't call anything in a loop yourself: the scheduler does that.

## Key ideas

- **Speeds are fractions.** 1.0 is full speed, 0.5 half, 0.25 quarter. That way the same code works on a fast or slow robot.
- **Time-based moves are approximate.** The robot needs a moment to speed up, so the distance is a bit less than speed x time.
- **A command that finishes stops the robot.** driveFor stops the wheels when its time is up.

## Try this

1. Predict the distance: 0.25 x 4.5 m/s x 2 s = 2.25 m. The terminal prints the robot's position every half second (the truth x= number). How close was your prediction?
2. Try forward=0.5. Does it go twice as far?
3. Try 1.0 seconds instead of 2.0.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
