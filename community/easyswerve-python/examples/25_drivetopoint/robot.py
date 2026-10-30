#!/usr/bin/env python3
"""
EXAMPLE: Drive to a spot by itself

Hold A and the robot drives itself to a spot on the field, steering around
obstacles. Let go and you're driving again. Spots are in meters from the
blue alliance corner.

On the robot: plug a controller into the Driver Station laptop,
and enable in Teleoperated. Drive somewhere, then hold A.

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

        # driveTo(x meters, y meters, facing degrees)
        self.controller.a().whileTrue(self.autos.driveTo(2.0, 4.0, 0))

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
