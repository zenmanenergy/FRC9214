#!/usr/bin/env python3
"""
SETUP STEP 1: Robot check-up (nothing moves)

Before anything moves, check that the code can talk to every part of the
drivetrain: all 8 SPARK MAXes, all 4 steering encoders, the navX gyro and the
battery. It also catches settings mistakes, like the same CAN ID typed twice
in robot_settings.py.

On the robot: deploy, leave the robot DISABLED, and read the console
(Driver Station or the VS Code terminal). Fix every PROBLEM line, from the top
down, deploy again, and repeat until everything says OK.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy
"""

import commands2
import wpilib

from easyswerve import Drivetrain


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()

        # No controller and no driving in this example: nothing moves.
        self.startTime = wpilib.Timer.getFPGATimestamp()
        self.lastReport = None
        self.autoCommand = None

    def robotPeriodic(self) -> None:
        """Runs 50 times a second in every mode."""
        now = wpilib.Timer.getFPGATimestamp()
        # Wait 3 seconds after startup (encoders and the navX need a moment),
        # then check every 10 seconds
        if now - self.startTime < 3.0:
            return
        if self.lastReport is not None and now - self.lastReport < 10.0:
            return
        self.lastReport = now
        self.printReport()

    def printReport(self) -> None:
        results = self.drivetrain.checkup()  # a list of (ok, message)
        problems = [text for ok, text in results if not ok]

        print("==== Robot check-up ====")
        for ok, text in results:
            label = "OK      " if ok else "PROBLEM "
            print(f"  {label} {text}")
        if problems:
            print(f"{len(problems)} problem(s). Fix the top one first: one mistake can cause several lines.")
        else:
            print("Everything OK. Next: examples/00_02_encodersbyhand")

        wpilib.SmartDashboard.putNumber("Checkup/problems", len(problems))

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        return None  # nothing moves in this example

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
