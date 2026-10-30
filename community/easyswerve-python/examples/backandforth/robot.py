#!/usr/bin/env python3
"""
EXAMPLE: Back and forth, again and again

.repeatedly() runs a command over and over. .withTimeout() stops it after
some time, so this goes back and forth for 8 seconds.

Try it in the simulator: click Autonomous.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
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
        backAndForth = commands2.SequentialCommandGroup(
            self.drivetrain.driveFor(1.0, forward=0.25),
            self.drivetrain.driveFor(1.0, forward=-0.25),
        )
        return backAndForth.repeatedly().withTimeout(8.0)

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
