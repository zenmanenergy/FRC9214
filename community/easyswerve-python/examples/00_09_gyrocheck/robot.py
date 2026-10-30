#!/usr/bin/env python3
"""
SETUP STEP 9: Check the gyro (IMU)

The navX gyro tells the robot which way it faces. This checks the three things
that go wrong with gyros: still calibrating, drifting while the robot sits
still, and turns that don't add up.

Try it in the simulator: click Disabled and watch the terminal (calibrating,
heading, drift), then click Autonomous for the turning test.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

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

        # Press Start: the way the robot faces now becomes heading 0
        self.controller.start().onTrue(self.drivetrain.resetHeading())

        self.driftStartTime = None
        self.driftStartHeading = 0.0
        self.lastPrint = 0.0
        self.autoCommand = None

    # ---- drift test: while disabled, the robot is still -----------------------------

    def disabledInit(self) -> None:
        """Runs once each time the robot is disabled: start a new drift test."""
        self.driftStartTime = None

    def disabledPeriodic(self) -> None:
        now = wpilib.Timer.getFPGATimestamp()
        heading = self.drivetrain.getHeading()

        # Don't measure drift until the navX has finished calibrating
        if self.drivetrain.gyroCalibrating():
            self.driftStartTime = None
        elif self.driftStartTime is None:
            self.driftStartTime = now
            self.driftStartHeading = heading

        if now - self.lastPrint < 2.0:
            return
        self.lastPrint = now

        if self.driftStartTime is None:
            print("Gyro: CALIBRATING - keep the robot still")
            return

        minutes = (now - self.driftStartTime) / 60.0
        drift = heading - self.driftStartHeading
        perMinute = drift / minutes if minutes > 0 else 0.0
        print(
            f"Gyro: heading {heading:7.2f} deg | drifted {drift:+.2f} deg in "
            f"{minutes * 60:4.0f} s ({perMinute:+.2f} deg per minute)"
        )
        wpilib.SmartDashboard.putNumber("Gyro/Drift (deg per minute)", perMinute)

    def robotPeriodic(self) -> None:
        """Runs 50 times a second in every mode: keep the dashboard current."""
        wpilib.SmartDashboard.putBoolean("Gyro/Calibrating", self.drivetrain.gyroCalibrating())
        wpilib.SmartDashboard.putNumber("Gyro/Heading (deg)", self.drivetrain.getHeading())
        wpilib.SmartDashboard.putNumber("Gyro/Turn rate (deg per s)", self.drivetrain.getTurnRate())

    # ---- turning test: in autonomous ----------------------------------------------------

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        # Turn to face each direction, then back to the start. If the gyro is
        # right, the robot ends facing exactly where it began.
        return commands2.SequentialCommandGroup(
            commands2.PrintCommand("Turning test: mark where the robot faces now"),
            self.drivetrain.turnTo(90),
            self.drivetrain.turnTo(180),
            self.drivetrain.turnTo(-90),
            self.drivetrain.turnTo(0),
            commands2.PrintCommand("Turning test done: is it facing the mark?"),
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
