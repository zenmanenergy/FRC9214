# 28: Driver and operator

## What it does

Use two controllers: the driver drives, and an operator runs everything else.

## On the robot

Copy this folder's robot.py over robot.py and deploy (py -m robotpy deploy). Plug two controllers into the Driver Station laptop; on the USB tab, drag the driver's to slot 0 and the operator's to slot 1. Clear some space and enable in **Teleoperated**.

## The code to look at

```python
self.operator = CommandXboxController(1)

# Operator buttons
self.operator.x().whileTrue(self.drivetrain.lockWheels())
```

- **CommandXboxController(1)** is the controller in Driver Station slot 1. The driver's is slot 0 (CONTROLLER_PORT in robot_settings.py).
- The operator's buttons are set up the same way as the driver's.

## Key ideas

- **Splitting jobs.** In matches, one person driving and another running mechanisms is common: each can focus on one thing.
- **Slot numbers** come from the order in the Driver Station's USB tab.

## Try this

1. Move the Start (reset heading) button from the operator back to the driver.
2. After example 31, give the intake buttons to the operator.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
