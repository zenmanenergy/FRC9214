# 27: Make the controller rumble

## What it does

Make the controller vibrate while B is held.

## On the robot

Copy this folder's robot.py over robot.py and deploy (py -m robotpy deploy). Plug an Xbox controller into the Driver Station laptop, enable in **Teleoperated**, and hold B. You'll feel it.

## The code to look at

```python
self.controller.b().whileTrue(                              # hold B to run this command:
    commands2.StartEndCommand(                              # does one thing at start, another at end
        lambda: self.rumble(1.0),                           # start: full rumble
        lambda: self.rumble(0.0),                           # end (B let go): rumble off
    )
)
```

- **StartEndCommand(start, end)** runs start when the command begins and end when it stops. Here: rumble on, rumble off.
- **lambda:** makes a small function in place. The command calls it later; without lambda, self.rumble(1.0) would run once, right now. rumble() (above makeAuto) does the actual shaking with setRumble.
- **1.0** is full strength; 0.0 is off.

## Key ideas

- **Commands don't have to use the drivetrain.** This one doesn't, so it doesn't interrupt driving: you can rumble and drive at the same time.
- **lambda** shows up a lot in robot code: it's how you hand an action to a command to do later.

## Try this

1. Use kLeftRumble or kRightRumble for one side only.
2. Try strength 0.3.
3. Make the controller rumble during autonomous, so the driver knows auto is still running.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
