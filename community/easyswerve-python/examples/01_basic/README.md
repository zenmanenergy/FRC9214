# 01: Basic robot

## What it does

The starting robot. Everything every robot.py in this folder has in common, plus driving with a controller, two buttons, and autos from the PathPlanner app. Read this one carefully: all the other examples change just one or two parts of it.

## On the robot

Copy this folder's robot.py over robot.py in the project folder and deploy (py -m robotpy deploy). In the dashboard (Elastic), pick **Test S-Curve** in the Auto chooser. Put the robot at one end of the practice carpet, facing the long way, centered side to side, with the back bumper about 20 cm from the edge. Have someone ready to disable, then enable in **Autonomous**. It drives an S-shaped path and prints "Hello from auto!" in the console at the end.

## The code to look at

```python
class MyRobot(commands2.TimedCommandRobot):                         # our robot: runs 50 times a second
    def robotInit(self) -> None:                                    # runs ONCE, when the robot code starts
```

```python
self.drivetrain = Drivetrain()                              # make the drivetrain (uses robot_settings.py)
port = robot_settings.CONTROLLER_PORT                       # Driver Station USB slot (usually 0)
self.controller = CommandXboxController(port)               # the driver's Xbox controller
```

```python
self.drivetrain.driveWithController(self.controller)        # left stick drives, right stick turns
```

```python
lock = self.drivetrain.lockWheels()                         # a command: wheels in an X (hard to push)
self.controller.rightBumper().whileTrue(lock)               # runs while the right bumper is held
```

```python
def makeAuto(self):                                             # builds what autonomous will do
    return self.autos.selected()                                # the auto picked in the dashboard's "Auto" chooser
```

```python
def autonomousInit(self) -> None:                               # runs once when enabled in Autonomous
    self.autoCommand = self.makeAuto()                          # build the auto (makeAuto, above)
    if self.autoCommand is not None:                            # if there is an auto...
        scheduler = commands2.CommandScheduler.getInstance()    # the part that runs commands
        scheduler.schedule(self.autoCommand)                    # start the auto
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
