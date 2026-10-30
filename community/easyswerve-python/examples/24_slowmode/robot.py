#!/usr/bin/env python3
"""
EXAMPLE: Slow mode

Hold the left trigger to drive at 30 % speed, for careful lining up. This
writes its own driving command instead of using driveWithController.

Try it in the simulator: plug in a controller, drag it onto Joystick slot 0,
and click Teleoperated. Drive, then hold the left trigger.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import commands2
from commands2.button import CommandXboxController
import wpimath

import robot_settings
from easyswerve import Drivetrain


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        self.drivetrain.setDefaultCommand(
            commands2.RunCommand(self.driveWithSlowMode, self.drivetrain)
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

    def driveWithSlowMode(self) -> None:
        # Full speed normally, 30 % while the left trigger is held
        speed = 0.3 if self.controller.getLeftTriggerAxis() > 0.5 else 1.0

        # Ignore tiny stick movements (stick drift)
        forward = -wpimath.applyDeadband(self.controller.getLeftY(), 0.1)
        left = -wpimath.applyDeadband(self.controller.getLeftX(), 0.1)
        turn = -wpimath.applyDeadband(self.controller.getRightX(), 0.1)

        self.drivetrain.drive(forward * speed, left * speed, turn * speed, True)
