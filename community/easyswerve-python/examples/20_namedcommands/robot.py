#!/usr/bin/env python3
"""
EXAMPLE: Named commands in PathPlanner autos

In the PathPlanner app, an auto can run a command by typing its name.
addCommand connects that name to real code. The Test S-Curve auto already
runs printHello at the end. In the app, try adding a Named Command block
called lockWheels to an auto.

On the robot: enable in Autonomous. Look for "Hello from auto!" in
the console.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                    # command-based robot library
from commands2.button import CommandXboxController                  # an Xbox controller with buttons

import robot_settings                                               # load robot_settings.py (your robot's settings)
from easyswerve import Autos, Drivetrain                            # drivetrain, and PathPlanner autos


class MyRobot(commands2.TimedCommandRobot):                         # our robot: runs 50 times a second
    def robotInit(self) -> None:                                    # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                              # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                       # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)               # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)        # left stick drives, right stick turns

        self.autos = Autos(self.drivetrain)                         # autos drawn in the PathPlanner app
        hello = commands2.PrintCommand("Hello from auto!")          # a command that prints a message
        self.autos.addCommand("printHello", hello)                  # autos can run it by this name
        lock = self.drivetrain.lockWheels().withTimeout(1.0)        # lock the wheels for 1 second
        self.autos.addCommand("lockWheels", lock)                   # autos can run it by this name

        self.autoCommand = None                                     # no auto yet (None = nothing)

    def makeAuto(self):                                             # builds what autonomous will do
        return self.autos.auto("Test S-Curve")                      # this auto uses both names (PathPlanner app)

    def autonomousInit(self) -> None:                               # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                          # build the auto (makeAuto, above)
        if self.autoCommand is not None:                            # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()    # the part that runs commands
            scheduler.schedule(self.autoCommand)                    # start the auto

    def teleopInit(self) -> None:                                   # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                            # if an auto is still running...
            self.autoCommand.cancel()                               # stop the auto: the driver takes over

    def testInit(self) -> None:                                     # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()        # stop every command that is running
