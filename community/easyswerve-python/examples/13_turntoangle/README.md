# 13: Turn to an angle

## What it does

Turn until the robot faces 90 degrees (a quarter turn left), using the gyro.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py). It ends facing left (90 degrees from where it started).

## The code to look at

```python
return self.drivetrain.turnTo(90)                           # turn to face 90 degrees (left), using the gyro
```

- **turnTo(90)** turns until the gyro says the robot faces 90 degrees, then stops.
- **Angles are absolute, not "turn by".** 0 is the direction the robot faced at the start (or after resetHeading). 90 is left, -90 is right, 180 is backward.

## Key ideas

- **The gyro (navX)** measures which way the robot faces.
- **PID control.** Inside turnTo, the turning speed comes from the error (how far from the target): P turns faster the farther off it is, I adds a push if it sits a little off for a while, D calms it down as it gets close. Far away: turn fast. Close: turn slowly, so it doesn't overshoot. The P, I and D are tuned by setup step 11.
- **Done means close and still.** turnTo finishes when it has stayed within 1 degree, and stopped turning, for a tenth of a second.
- **The same idea keeps the robot straight.** Heading hold (example 36) uses the same heading PID: while the robot drives without turning, it turns back a little whenever it has drifted off.

## Try this

1. Try turnTo(-90) and turnTo(180).
2. Put turnTo(90) twice in a SequentialCommandGroup. Why does the second one do nothing? (Hint: absolute angles.)
3. Curious how it works? Find def turnTo in easyswerve/drivetrain/drivetrain.py.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
