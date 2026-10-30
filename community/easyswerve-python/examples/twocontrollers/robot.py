#!/usr/bin/env python3
"""
EXAMPLE: Driver and operator

Many teams use two controllers: the driver drives, the operator runs the
rest. The operator's controller is in Driver Station slot 1.

Try it in the simulator: plug in two controllers, drag them onto Joystick
slots 0 and 1, and click Teleoperated.

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

        self.operator = CommandXboxController(1)

        # Operator buttons
        self.operator.x().whileTrue(self.drivetrain.lockWheels())
        self.operator.start().onTrue(self.drivetrain.resetHeading())

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
