#!/usr/bin/env python3
"""
EXAMPLE: Pick an auto on the dashboard

A SendableChooser shows a drop-down list on the dashboard. The drive team
picks an auto before the match, without changing code.

On the robot: open Elastic (connected to the robot), pick an auto
in the Auto chooser, then enable in Autonomous.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy
"""

import commands2
from commands2.button import CommandXboxController
import wpilib

import robot_settings
from easyswerve import Drivetrain


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        # The list of autos. Each entry is a function that builds the auto.
        self.chooser = wpilib.SendableChooser()
        self.chooser.setDefaultOption("Drive forward", self.driveForward)
        self.chooser.addOption("Spin", self.spin)
        self.chooser.addOption("Do nothing", self.doNothing)
        wpilib.SmartDashboard.putData("Auto", self.chooser)

        self.autoCommand = None

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        buildAuto = self.chooser.getSelected()
        return buildAuto()

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

    def driveForward(self):
        return self.drivetrain.driveFor(2.0, forward=0.25)

    def spin(self):
        return self.drivetrain.driveFor(3.0, turn=0.3)

    def doNothing(self):
        return None
