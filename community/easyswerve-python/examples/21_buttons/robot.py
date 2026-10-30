#!/usr/bin/env python3
"""
EXAMPLE: More buttons

Buttons run commands. whileTrue: while held. onTrue: once when pressed.

Try it in the simulator: plug in a controller, drag it onto Joystick slot 0,
and click Teleoperated. Press A, B, X, Y and Start.

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

        # A: stop the wheels
        self.controller.a().onTrue(self.drivetrain.stop())

        # X (hold): lock the wheels
        self.controller.x().whileTrue(self.drivetrain.lockWheels())

        # Y: turn to face away from the driver
        self.controller.y().onTrue(self.drivetrain.turnTo(0))

        # B: turn to face the driver
        self.controller.b().onTrue(self.drivetrain.turnTo(180))

        # Start: the way the robot faces now becomes "forward"
        self.controller.start().onTrue(self.drivetrain.resetHeading())

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
