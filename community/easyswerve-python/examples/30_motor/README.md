# 30: Spin a motor with a button

## What it does

The simplest mechanism: one motor on a SPARK MAX. Hold A to spin it one way, B the other.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). Plug an Xbox controller into the Driver Station laptop (it should be in slot 0 on the USB tab), clear some space, and enable in **Teleoperated**. Hold A or B. Only use this once a motor with this CAN ID is wired up and free to spin.

## The code to look at

```python
MOTOR_CAN_ID = 20  # change to your motor's CAN ID
```

```python
self.motor = SparkMax(MOTOR_CAN_ID, SparkMax.MotorType.kBrushless)
```

```python
self.controller.a().whileTrue(
    commands2.StartEndCommand(lambda: self.motor.set(0.5), lambda: self.motor.set(0))
)
```

- **SparkMax(CAN ID, brushless)** connects to one motor controller. Each SPARK MAX on the robot has its own CAN ID, set in REV Hardware Client.
- **motor.set(0.5)** runs the motor at half power. -0.5 is half power backward, 0 stops it.
- **StartEndCommand** (from 27) starts the motor when A is pressed and stops it when A is let go.
- **MOTOR_CAN_ID at the top** is a constant: a named number that's easy to find and change.

## Key ideas

- **Every mechanism starts here:** a motor, and commands that run it.
- **Always stop what you start.** The end part of StartEndCommand makes sure the motor never keeps running after you let go.

## Try this

1. Change the speed to 0.25.
2. Use toggleOnTrue on A: press to start, press again to stop.
3. Why might two buttons that both control this motor be a problem? Example 31 fixes it.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
