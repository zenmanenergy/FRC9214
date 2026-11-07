# 05: Spin in place

## What it does

Turn in a circle without moving: 3 seconds of spinning counterclockwise.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py).

## The code to look at

```python
return self.drivetrain.driveFor(3.0, turn=0.3)              # spin in place for 3 s (+ = counterclockwise)
```

- **turn=0.3** spins at 30 % of the top turning speed. Positive is counterclockwise (seen from above), negative is clockwise.
- Top turning speed is MAX_TURN_DEG_PER_S in robot_settings.py: 180 degrees per second to start. So 0.3 is 54 degrees per second.

## Key ideas

- **Counterclockwise is positive.** This is the math convention (like angles on the unit circle) and WPILib uses it everywhere.
- **The wheels point in a circle** around the robot's center so the robot spins in place.

## Try this

1. Work out how long one full turn takes at turn=0.3: 360 / 108 = 3.33 seconds. Change 3.0 to 3.33. Does it end facing where it started? (Put a tape line along one side before you start.)
2. Try turn=-0.3 to spin clockwise.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
