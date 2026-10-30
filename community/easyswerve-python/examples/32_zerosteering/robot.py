#!/usr/bin/env python3
"""
EXAMPLE: Zero the steering encoders

Teach the robot which way "straight ahead" is for each wheel. Do this once on
a new robot, and again whenever a module or encoder is taken apart.

Try it in the simulator: click Disabled, then press Back on the controller (or
the Calibrate button on the dashboard). The terminal prints each wheel's angle
every 2 seconds while disabled.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import math

import commands2
import wpilib
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

        # Back button: save the current wheel angles as "straight ahead".
        # Only while DISABLED, with the hex keys in all four modules.
        self.controller.back().onTrue(self.drivetrain.zeroSteeringCommand())

        self.lastPrint = 0.0
        self.autoCommand = None

    def disabledPeriodic(self) -> None:
        """Runs 50 times a second while disabled: show each wheel's angle."""
        now = wpilib.Timer.getFPGATimestamp()
        if now - self.lastPrint < 2.0:
            return
        self.lastPrint = now

        # 0 = straight ahead. With the hex keys in, after zeroing, expect
        # FL 45, FR -45, RL 135, RR -135: REV's code expects the hex-key
        # position to be an X, not straight ahead.
        angles = []
        for name, module in self.drivetrain.modules:
            # The wheel angle relative to the robot, in degrees
            degrees = math.degrees(module.getState().angle.radians())
            angles.append(f"{name} {degrees:6.1f}")
        print("Wheel angles: " + "   ".join(angles))

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        # Check: drive slowly forward. Every wheel should point straight ahead.
        return self.drivetrain.driveFor(1.5, forward=0.15)

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
