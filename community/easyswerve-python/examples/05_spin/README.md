# 05: Spin in place

## What it does

Turn in a circle without moving: 3 seconds of spinning counterclockwise.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**.

## The code to look at

```python
return self.drivetrain.driveFor(3.0, turn=0.3)
```

- **turn=0.3** spins at 30 % of the top turning speed. Positive is counterclockwise (seen from above), negative is clockwise.
- Top turning speed is MAX_TURN_DEG_PER_S in robot_settings.py: 360 degrees per second. So 0.3 is 108 degrees per second.

## Key ideas

- **Counterclockwise is positive.** This is the math convention (like angles on the unit circle) and WPILib uses it everywhere.
- **The wheels point in a circle** around the robot's center so the robot spins in place.

## Try this

1. Work out how long one full turn takes at turn=0.3: 360 / 108 = 3.33 seconds. Change 3.0 to 3.33. Does it end facing where it started? (h= in the status line is the heading.)
2. Try turn=-0.3 to spin clockwise.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
