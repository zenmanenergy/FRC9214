#!/usr/bin/env python3
"""
EXAMPLE: Run one auto from the PathPlanner app

Autos drawn in the PathPlanner app are saved in deploy/pathplanner/autos/.
This runs one of them by name, with no dashboard needed.

On the robot: enable in Autonomous.

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
        self.autos.addCommand("printHello", commands2.PrintCommand("Hello from auto!"))

        self.autoCommand = None

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        # The name as it appears in the PathPlanner app
        return self.autos.auto("Drive Forward")

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
