#!/usr/bin/env python3
"""
EXAMPLE: Point the robot with the D-pad

The D-pad (POV) is four more buttons. Each one turns the robot to face a
direction.

Try it in the simulator: plug in a controller, drag it onto Joystick slot 0,
and click Teleoperated. Press the D-pad.

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

        self.controller.povUp().onTrue(self.drivetrain.turnTo(0))
        self.controller.povLeft().onTrue(self.drivetrain.turnTo(90))
        self.controller.povDown().onTrue(self.drivetrain.turnTo(180))
        self.controller.povRight().onTrue(self.drivetrain.turnTo(-90))

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
