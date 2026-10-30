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
"""

import commands2
from commands2.button import CommandXboxController

import robot_settings
from easyswerve import Autos, Drivetrain


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        self.autos = Autos(self.drivetrain)

        # name in the app  -> what it does
        self.autos.addCommand("printHello", commands2.PrintCommand("Hello from auto!"))
        self.autos.addCommand("lockWheels", self.drivetrain.lockWheels().withTimeout(1.0))

        self.autoCommand = None

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        return self.autos.auto("Test S-Curve")

    def autonomousInit(self) -> None:
        self.autoCommand = self.makeAuto()
        if self.autoCommand is not None:
            commands2.CommandScheduler.getInstance().schedule(self.autoCommand)

    def teleopInit(self) -> None:
        # Stop the auto when the driver takes over
        if self.autoCommand is not None:
            self.autoCommand.cancel()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()
