# 27: Make the controller rumble

## What it does

Make the controller vibrate while B is held.

## On the robot

Copy this folder's robot.py over robot.py and deploy (py -m robotpy deploy). Plug an Xbox controller into the Driver Station laptop, enable in **Teleoperated**, and hold B. You'll feel it.

## The code to look at

```python
self.controller.b().whileTrue(
    commands2.StartEndCommand(
        lambda: self.controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
        lambda: self.controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0),
    )
)
```

- **StartEndCommand(start, end)** runs start when the command begins and end when it stops. Here: rumble on, rumble off.
- **lambda:** makes a small function in place. The command calls it later; without lambda, setRumble would run once, right now.
- **1.0** is full strength; 0.0 is off.

## Key ideas

- **Commands don't have to use the drivetrain.** This one doesn't, so it doesn't interrupt driving: you can rumble and drive at the same time.
- **lambda** shows up a lot in robot code: it's how you hand an action to a command to do later.

## Try this

1. Use kLeftRumble or kRightRumble for one side only.
2. Try strength 0.3.
3. Make the controller rumble during autonomous, so the driver knows auto is still running.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
