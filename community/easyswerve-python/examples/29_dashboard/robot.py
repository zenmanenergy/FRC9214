#!/usr/bin/env python3
"""
EXAMPLE: Show numbers on the dashboard

robotPeriodic runs 50 times a second in every mode. SmartDashboard puts
values on the dashboard, where you can watch them live.

On the robot: open Elastic (connected to the robot), enable in
Autonomous, and add the My Robot values to a tab.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy
"""

import commands2
from commands2.button import CommandXboxController
import wpilib

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
        return self.drivetrain.driveFor(3.0, forward=0.3, turn=0.15)

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

    def robotPeriodic(self) -> None:
        pose = self.drivetrain.getPose()
        wpilib.SmartDashboard.putNumber("My Robot/X (m)", pose.X())
        wpilib.SmartDashboard.putNumber("My Robot/Y (m)", pose.Y())
        wpilib.SmartDashboard.putNumber("My Robot/Heading (deg)", self.drivetrain.getHeading())
        wpilib.SmartDashboard.putBoolean("My Robot/Enabled", wpilib.DriverStation.isEnabled())
