# 36: Heading hold (keep facing the same way)

## What it does

Keep the robot facing the same way while it drives. No two wheels are exactly the same: one has more friction, or a tighter bearing. Driving "straight", those small differences slowly turn the robot. Heading hold fixes that: whenever the robot drives without being told to turn, the navX gyro keeps it facing the way it faced when the turning stopped. It's on by default (HEADING_HOLD in robot_settings.py); this example switches it off and on so you can see what it does.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Put the robot in the middle of the practice carpet, facing the long way, have someone ready to disable (Enter disables, space bar is the emergency stop), then enable in **Autonomous**. The moves are sized to fit the carpet (CARPET_LENGTH_M and CARPET_WIDTH_M in robot_settings.py). It drives forward 2 seconds (less on a short carpet) with heading hold OFF, then back with it ON, and prints how much the robot turned each time. On a well-built robot the difference is small; to make it big, put a strip of carpet or tape under one side, or have someone gently push a front corner while it drives. In Teleoperated, press **A** to switch heading hold off and on.

## The code to look at

```python
dt.headingHold = not dt.headingHold                            # not True = False, not False = True
```

```python
turned = (now - self.startHeading + 180.0) % 360.0 - 180.0     # the change, -180 to 180
```

```python
commands2.InstantCommand(lambda: self.startLeg(False)),    # 1. hold OFF
dt.driveFor(seconds, forward=0.25),                        # 2. drive forward 2 s
commands2.InstantCommand(self.endLeg),                     # 3. how much did it turn?
```

- **drivetrain.headingHold** is True or False: heading hold on or off. It starts as HEADING_HOLD from robot_settings.py, and code can change it any time.
- **not** flips True to False and False to True.
- **The angle math** ((now - start + 180) % 360 - 180) gives the change between two headings from -180 to 180, so going from 179 to -179 degrees counts as +2, not -358.
- **InstantCommand** runs one function once, so it can sit between the drives in a SequentialCommandGroup.

## Key ideas

- **Feedback again.** Heading hold uses the same heading PID as turnTo (example 13), tuned by setup step 11: the gyro measures how far the robot has turned off, and the code turns it back, faster the farther off it is (never faster than 70 degrees per second).
- **It lets go when you steer.** The moment the turn stick (or code) asks to turn, heading hold stops. When the turning stops, it waits until the robot has stopped rotating, then holds that new direction.
- **Not when standing still.** It only corrects while the robot is driving, so the wheels don't twitch while it sits.
- **PathPlanner does its own.** Paths from the PathPlanner app control the heading themselves, so heading hold isn't used there.
- **It needs a good gyro.** A navX that drifts makes heading hold slowly turn the robot to follow the drift. Setup step 9 (00_09_gyrocheck) checks for that.

## Try this

1. Push the robot's front corner while it drives with heading hold ON. What does it do? Then press A to switch it off and try again.
2. In robot_settings.py, set HEADING_HOLD = False and drive around in Teleoperated. Does driving straight across the field feel different?
3. Run setup step 11 (00_11_autotuneposition) and read the heading PID it keeps: P, I and D. Then run this example again. Does the robot turn less with heading hold ON?

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
