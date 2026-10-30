# 01: Basic robot

## What it does

The starting robot. Everything every robot.py in this folder has in common, plus driving with a controller, two buttons, and autos from the PathPlanner app. Read this one carefully: all the other examples change just one or two parts of it.

## Run it

Copy this folder's robot.py over robot.py in the project folder, run py -m robotpy sim, and click **Autonomous**. The robot drives the S-Curve auto and prints "Hello from auto!" at the end.

## The code to look at

```python
class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
```

```python
self.drivetrain = Drivetrain()
self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)
```

```python
self.drivetrain.driveWithController(self.controller)
```

```python
self.controller.rightBumper().whileTrue(self.drivetrain.lockWheels())
```

```python
def makeAuto(self):
    """Builds the command that runs in autonomous."""
    # The auto picked in the dashboard's "Auto" chooser
    return self.autos.selected()
```

```python
def autonomousInit(self) -> None:
    self.autoCommand = self.makeAuto()
    if self.autoCommand is not None:
        commands2.CommandScheduler.getInstance().schedule(self.autoCommand)
```

- **MyRobot** is your robot. RobotPy creates it when the robot turns on.
- **robotInit** runs once, at power-on. This is where you build the robot: the drivetrain, the controller, the buttons.
- **Drivetrain()** is the whole swerve drive: 4 modules, the gyro, position tracking, and the autotune buttons. One line.
- **driveWithController** makes the sticks drive the robot whenever nothing else is using the drivetrain.
- **A button line** reads like a sentence: "right bumper, while held, lock the wheels".
- **makeAuto** builds the command for autonomous. Every example changes this function.
- **autonomousInit** runs when autonomous starts: it asks makeAuto for the command and schedules it (starts it running). **teleopInit** cancels it when the driver takes over.

## Key ideas

- **Commands** are things the robot does: drive, lock the wheels, print a message. Buttons and autos start commands.
- **The scheduler** runs every scheduled command 50 times a second until the command says it's finished.
- **Modes**: autonomous (the robot drives itself), teleoperated (the driver drives), test (used by autotune), disabled (nothing moves).
- **robot_settings.py** holds the wiring and measurements, so robot.py only describes behavior.

## Try this

1. Change rightBumper to leftBumper. Run teleop and check the new button works.
2. Change the message in the printHello line and run the auto again.
3. Find each of robotInit, makeAuto, autonomousInit and teleopInit in another example. They're always there.

To go back to the starting robot, copy 01_basic/robot.py over robot.py.
