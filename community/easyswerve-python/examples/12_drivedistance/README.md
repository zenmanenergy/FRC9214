# 12: Drive a distance

## What it does

Drive exactly 2 meters (or less, to fit the practice carpet), measured with the wheel encoders, instead of driving for a time.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py). The robot stops that far from where it started, within about a centimeter. Put a tape mark at the front bumper first and measure.

## The code to look at

```python
meters = carpet.fitMeters(2.0)                              # 2 m, or less to stay on the carpet
return self.drivetrain.driveDistance(meters, speed=0.3)     # drive exactly that far (wheel encoders), speed 0.3
```

- **carpet.fitMeters(2.0)** is 2 m, or less if the practice carpet is too short (easyswerve/carpet.py works out the room from CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py). On a big floor, set CARPET_LENGTH_M = None and it is always 2 m.
- **driveDistance(meters)** keeps driving until the robot's position says it has gone that far, slowing down as it gets close.
- **speed=0.3** is the top speed on the way. It changes how fast it gets there, not how far it goes.

## Key ideas

- **Sensors vs. timing.** driveFor (02) guesses distance from time. driveDistance measures it. Measuring is more accurate, especially if the battery is low or the robot is heavy.
- **Feedback.** The robot checks a sensor every loop and decides whether to keep going. This idea (measure, compare, act) is at the heart of robot control.
- **Two loops.** Inside each SPARK MAX, a speed PID makes each wheel go the speed it's asked for. driveDistance adds a position PID around it: every loop it compares where the robot is with the target spot and asks for a speed. Far away: fast (up to the speed you gave). Close: slower and slower, so it arrives gently instead of braking hard at the line and rolling past. If it does roll past, the PID drives it back.
- **Why not just stop at 2 m?** A speed PID only knows speed, not where to stop. If you ask for full speed until the line and then 0, the robot needs some distance to brake, and that distance grows with speed.
- **The gains are tuned, not guessed.** The position PID's P, I and D come from setup step 11 (00_11_autotuneposition), which measures your robot and tries two sets of gains on real moves. It also holds the heading, so the robot doesn't turn on the way.

## Try this

1. Change speed to 0.6 and then 0.15. Does the distance stay 2 m? Compare with changing the speed in 02.
2. Run setup step 11 again and read the position PID it keeps. Then compare: how close does driveDistance stop now, measured with a tape?
3. Drive backward 1 meter: driveDistance(-1.0).

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
