#!/usr/bin/env python3
"""
EXAMPLE: Drive a path

A path is one drawn line in the PathPlanner app. drivePath sets the robot's
position to the start of the path and drives it. On the real robot, place
the robot at the path's start first.

Try it in the simulator: click Autonomous.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
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

        self.autoCommand = None

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        return self.autos.drivePath("S-Curve")

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
