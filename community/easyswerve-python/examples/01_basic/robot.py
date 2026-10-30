#!/usr/bin/env python3
"""
EXAMPLE: Basic robot (the starting robot.py)

Driving with a controller, two buttons, and autos from the PathPlanner app.
Copy this back over robot.py to start over.

On the robot: in the dashboard (Elastic), pick "Test S-Curve" in the Auto
chooser, then enable in Autonomous. The robot drives an S-shaped path, about
3 m forward and 1 m to the side: clear the floor first.

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

        # Hold the right bumper: lock the wheels in an X (hard to push)
        self.controller.rightBumper().whileTrue(self.drivetrain.lockWheels())

        # Press Start: the way the robot faces now becomes "forward"
        self.controller.start().onTrue(self.drivetrain.resetHeading())

        # Autos drawn in the PathPlanner app (deploy/pathplanner/)
        self.autos = Autos(self.drivetrain)

        # Commands your autos can run by name (type the name into the app)
        self.autos.addCommand("printHello", commands2.PrintCommand("Hello from auto!"))

        self.autoCommand = None

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        # The auto picked in the dashboard's "Auto" chooser
        return self.autos.selected()

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
