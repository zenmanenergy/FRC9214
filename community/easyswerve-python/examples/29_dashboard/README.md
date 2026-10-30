# 29: Show numbers on the dashboard

## What it does

Put the robot's position and heading on the dashboard, updated live.

## On the robot

Copy this folder's robot.py over robot.py and deploy (py -m robotpy deploy). Open Elastic (connected to the robot) and add the My Robot values to a tab. Clear space, someone ready to disable, then enable in **Autonomous**.

## The code to look at

```python
def robotPeriodic(self) -> None:
    pose = self.drivetrain.getPose()
    wpilib.SmartDashboard.putNumber("My Robot/X (m)", pose.X())
```

- **robotPeriodic** runs 50 times a second in every mode, even disabled.
- **getPose()** is where the robot thinks it is: X, Y (meters) and rotation.
- **putNumber(name, value)** sends a number to the dashboard. The slash in "My Robot/X (m)" groups values into a folder.

## Key ideas

- **Seeing inside the robot.** Dashboards are how teams check what the code thinks while the robot runs.
- **Periodic methods** (robotPeriodic, and autonomousPeriodic and teleopPeriodic if you add them) run every loop.

## Try this

1. Add a counter: start at 0 in robotInit, add 1 in robotPeriodic, and put it on the dashboard. How fast does it count?
2. Put the battery voltage on the dashboard: wpilib.RobotController.getBatteryVoltage().

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
