# 13: Turn to an angle

## What it does

Turn until the robot faces 90 degrees (a quarter turn left), using the gyro.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**. Check h= in the status line: it ends near 90.

## The code to look at

```python
return self.drivetrain.turnTo(90)
```

- **turnTo(90)** turns until the gyro says the robot faces 90 degrees, then stops.
- **Angles are absolute, not "turn by".** 0 is the direction the robot faced at the start (or after resetHeading). 90 is left, -90 is right, 180 is backward.

## Key ideas

- **The gyro (navX)** measures which way the robot faces.
- **Proportional control.** Inside turnTo, the turning speed is the error (how far from the target) times a number. Far away: turn fast. Close: turn slowly. That way it doesn't overshoot much. This is the P in PID.
- **Done means close and still.** turnTo finishes when it's within 2 degrees and has stopped turning.

## Try this

1. Try turnTo(-90) and turnTo(180).
2. Put turnTo(90) twice in a SequentialCommandGroup. Why does the second one do nothing? (Hint: absolute angles.)
3. Curious how it works? Find def turnTo in easyswerve/drivetrain/drivetrain.py.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
