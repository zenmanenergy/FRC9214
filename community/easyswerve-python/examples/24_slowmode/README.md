# 24: Slow mode

## What it does

Hold the left trigger to drive at 30 % speed for careful lining up.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Plug an Xbox controller into the Driver Station laptop (it should be in slot 0 on the USB tab), clear some space, and enable in **Teleoperated**. Drive, then hold the left trigger.

## The code to look at

```python
self.drivetrain.setDefaultCommand(
    commands2.RunCommand(self.driveWithSlowMode, self.drivetrain)
)
```

```python
speed = 0.3 if self.controller.getLeftTriggerAxis() > 0.5 else 1.0
```

```python
forward = -wpimath.applyDeadband(self.controller.getLeftY(), 0.1)
```

```python
self.drivetrain.drive(forward * speed, left * speed, turn * speed, True)
```

- **setDefaultCommand** sets what the drivetrain does when no other command is using it. driveWithController (01) does this for you; here it's written out.
- **RunCommand** calls driveWithSlowMode 50 times a second, forever.
- **getLeftTriggerAxis()** reads how far the trigger is pressed, from 0 to 1.
- **applyDeadband** ignores tiny stick movements, so a stick that doesn't center perfectly doesn't creep the robot.
- **The minus sign:** pushing a stick forward reads as negative, so it's flipped.
- **drive(forward, left, turn, True)** is the basic driving method: three speeds and field-relative.

## Key ideas

- **The default command** is how teleop driving works under the hood.
- **Reading the controller directly** lets you shape driving however you like.
- **One-line if:** a if condition else b picks between two values.

## Try this

1. Change 0.3 to 0.5.
2. Make slow mode only slow the turning, not the driving.
3. Add a turbo: speed = 1.0 normally, but limit to 0.7 unless the right trigger is held.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
