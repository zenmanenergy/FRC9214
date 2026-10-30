# 12: Drive a distance

## What it does

Drive exactly 2 meters, measured with the wheel encoders, instead of driving for a time.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**. The robot stops about 2 meters from where it started.

## The code to look at

```python
# driveDistance(meters, speed)
return self.drivetrain.driveDistance(2.0, speed=0.3)
```

- **driveDistance(2.0)** keeps driving until the robot's position says it has gone 2 meters, then stops.
- **speed=0.3** only changes how fast it gets there, not how far it goes.

## Key ideas

- **Sensors vs. timing.** driveFor (02) guesses distance from time. driveDistance measures it. Measuring is more accurate, especially if the battery is low or the robot is heavy.
- **Feedback.** The robot checks a sensor every loop and decides whether to keep going. This idea (measure, compare, act) is at the heart of robot control.
- **Small overshoot.** It stops when it reaches 2 m, then coasts a little. That's normal.

## Try this

1. Change speed to 0.5 and then 0.15. Does the distance stay about 2 m? Compare with changing the speed in 02.
2. Drive backward 1 meter: driveDistance(-1.0).

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
