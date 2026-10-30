#!/usr/bin/env python3
"""
EXAMPLE: Spin a motor with a button

The simplest mechanism: one motor on a SPARK MAX. Hold A to spin it one way,
B the other way. Set MOTOR_CAN_ID to your motor's CAN ID. On the real robot,
only use this once that motor is wired up.

Try it in the simulator: plug in a controller, drag it onto Joystick slot 0,
and click Teleoperated. Hold A or B; the motor shows in the simulator's
SPARK MAX list.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import commands2
from commands2.button import CommandXboxController
from rev import SparkMax

import robot_settings
from easyswerve import Drivetrain


MOTOR_CAN_ID = 20  # change to your motor's CAN ID


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        self.motor = SparkMax(MOTOR_CAN_ID, SparkMax.MotorType.kBrushless)

        # Hold A: spin forward at 50 %. Let go: stop.
        self.controller.a().whileTrue(
            commands2.StartEndCommand(lambda: self.motor.set(0.5), lambda: self.motor.set(0))
        )
        # Hold B: spin backward
        self.controller.b().whileTrue(
            commands2.StartEndCommand(lambda: self.motor.set(-0.5), lambda: self.motor.set(0))
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
