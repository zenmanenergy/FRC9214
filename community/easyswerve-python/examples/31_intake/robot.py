#!/usr/bin/env python3
"""
EXAMPLE: An intake mechanism

A mechanism as a subsystem (a class), with commands it can do. The buttons
use them in teleop, and the auto drives forward while the intake runs.

On the robot: enable in Autonomous, and watch the console: intake
on, then off when the drive ends.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy
"""

import commands2
from commands2.button import CommandXboxController
from rev import SparkMax

import robot_settings
from easyswerve import Drivetrain


INTAKE_CAN_ID = 20  # change to your intake's CAN ID
INTAKE_SPEED = 0.6  # 0 to 1 (fraction of full power)


class Intake(commands2.Subsystem):
    """A one-motor intake."""

    def __init__(self) -> None:
        super().__init__()
        self.motor = SparkMax(INTAKE_CAN_ID, SparkMax.MotorType.kBrushless)

    def runIn(self) -> commands2.Command:
        """Pull in while running, stop when done."""
        return commands2.StartEndCommand(self.start, self.stop, self)

    def runOut(self) -> commands2.Command:
        """Spit out while running, stop when done."""
        return commands2.StartEndCommand(self.reverse, self.stop, self)

    def start(self) -> None:
        print("intake on")
        self.motor.set(INTAKE_SPEED)

    def reverse(self) -> None:
        print("intake reversing")
        self.motor.set(-INTAKE_SPEED)

    def stop(self) -> None:
        print("intake off")
        self.motor.set(0)


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        self.intake = Intake()

        # Hold A: pull in. Hold B: spit out.
        self.controller.a().whileTrue(self.intake.runIn())
        self.controller.b().whileTrue(self.intake.runOut())

        self.autoCommand = None

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        # ParallelDeadlineGroup: the first command is the deadline. The others
        # run alongside it and are stopped when it finishes.
        return commands2.ParallelDeadlineGroup(
            self.drivetrain.driveFor(2.0, forward=0.2),
            self.intake.runIn(),
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
