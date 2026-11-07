# 02: Drive forward

## What it does

The simplest possible auto: drive straight forward for 2 seconds at quarter speed, then stop. If the practice carpet is too short for 2 seconds, it stops sooner.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py).

Watch: Put a tape mark at the front bumper before you start, so you can measure the distance afterwards.

## The code to look at

```python
seconds = carpet.fitSeconds(2.0, forward=0.25)              # 2 s, or less to stay on the carpet
return self.drivetrain.driveFor(seconds, forward=0.25)      # 2 seconds forward at 0.25 (quarter) speed
```

- **carpet.fitSeconds(2.0, forward=0.25)** is 2 seconds, or less if driving that long at that speed would take the robot off the practice carpet. easyswerve/carpet.py works it out from CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py, the robot's size and MAX_SPEED_MPS. On a big floor, set CARPET_LENGTH_M = None and it is always 2 seconds. When it shortens a move, the console says so.
- **driveFor** is a command: drive at a fixed speed for some seconds, then stop.
- **2.0** is the time in seconds.
- **forward=0.25** is the speed as a fraction of top speed. Top speed is MAX_SPEED_MPS in robot_settings.py (1.5 m/s to start), so 0.25 is about 0.38 m/s.
- **makeAuto returns the command**, and autonomousInit starts it. You don't call anything in a loop yourself: the scheduler does that.

## Key ideas

- **Speeds are fractions.** 1.0 is full speed, 0.5 half, 0.25 quarter. That way the same code works on a fast or slow robot.
- **Time-based moves are approximate.** The robot needs a moment to speed up, so the distance is a bit less than speed x time.
- **A command that finishes stops the robot.** driveFor stops the wheels when its time is up.

## Try this

1. Predict the distance: speed x MAX_SPEED_MPS x seconds. At MAX_SPEED_MPS = 1.5 that is 0.25 x 1.5 m/s x 2 s = 0.75 m. Measure how far it really went with a tape measure. How close was your prediction?
2. Try forward=0.5. Does it go twice as far? (If the console says the move was shortened, the carpet is the limit.)
3. Try 1.0 seconds instead of 2.0.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
