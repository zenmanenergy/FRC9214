#!/usr/bin/env python3
"""
EXAMPLE: Make the controller rumble

StartEndCommand runs one thing when it starts and another when it ends.
Here: rumble on while B is held, off when let go.

Try it in the simulator: plug in a controller, click Teleoperated, and hold
B (you'll feel it).

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import commands2
from commands2.button import CommandXboxController
from wpilib.interfaces import GenericHID

import robot_settings
from easyswerve import Drivetrain


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        self.controller.b().whileTrue(
            commands2.StartEndCommand(
                lambda: self.controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
                lambda: self.controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0),
            )
        )

        self.autoCommand = None

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        # No auto in this example
        return None

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
