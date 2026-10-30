#!/usr/bin/env python3
"""
EXAMPLE: Wait, then drive

WaitCommand pauses. Useful when another robot needs to move out of the way
first.

On the robot: enable in Autonomous. Nothing happens for 3 seconds,
then it drives.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy
"""

import commands2
from commands2.button import CommandXboxController

import robot_settings
from easyswerve import Drivetrain


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        self.autoCommand = None

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        return commands2.SequentialCommandGroup(
            commands2.WaitCommand(3.0),
            self.drivetrain.driveFor(2.0, forward=0.25),
        )

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
