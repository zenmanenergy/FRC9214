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

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import re

import commands2                                                            # command-based robot library
import wpilib                                                               # WPILib: the library all FRC robots use

from easyswerve import Drivetrain                                           # the swerve drivetrain (4 modules)


def withoutNumbers(text: str) -> str:                                       # "battery 11.8 V" -> "battery # V"
    return re.sub(r"[0-9.]+", "#", text)                                    # so a reading going 11.8 -> 11.9 isn't "new"


class MyRobot(commands2.TimedCommandRobot):                                 # our robot: runs 50 times a second
    def robotInit(self) -> None:                                            # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                      # no controller here: nothing moves
        self.startTime = wpilib.Timer.getFPGATimestamp()                    # the time now (seconds)
        self.lastCheck = 0.0                                                # when we last checked
        self.lastResults = None                                             # the last check's problems (None = no check yet)
        self.autoCommand = None                                             # no auto yet (None = nothing)

    def robotPeriodic(self) -> None:                                        # runs 50 times a second, in every mode
        now = wpilib.Timer.getFPGATimestamp()                               # the time now, in seconds
        if now - self.startTime < 3.0:                                      # in the first 3 seconds (sensors are starting)...
            return                                                          # ...wait
        if now - self.lastCheck < 2.0:                                      # checked less than 2 seconds ago?
            return                                                          # then wait
        self.lastCheck = now                                                # remember that we're checking now
        results = self.drivetrain.checkup()                                 # a list of (ok, message) pairs
        problems = {withoutNumbers(t): t for ok, t in results if not ok}    # this check's problems
        if self.lastResults is None:                                        # the first check?
            self.printReport(results)                                       # print everything once
        else:                                                               # after that, print only what changed:
            for key, text in problems.items():                              # each problem now...
                if key not in self.lastResults:                             # ...that wasn't there before
                    print("NEW PROBLEM: " + text)
            for key, text in self.lastResults.items():                      # each problem before...
                if key not in problems:                                     # ...that is gone now
                    print("FIXED: " + text)
        self.lastResults = problems                                         # remember this check's problems for next time
        board = wpilib.SmartDashboard                                       # the dashboard
        board.putNumber("Checkup/problems", len(problems))                  # show the count there

    def printReport(self, results) -> None:                                 # prints every line: OK or PROBLEM
        problems = [text for ok, text in results if not ok]                 # just the messages that are not ok

        print("==== Robot check-up ====")
        for ok, text in results:                                            # repeat for each (ok, message)
            label = "OK      " if ok else "PROBLEM "                        # pick the label
            print("  " + label + " " + text)                                # e.g. "  OK       navX connected"
        if problems:                                                        # any problems at all?
            count = str(len(problems))                                      # how many, as text
            print(count + " problem(s). Fix the top one first:")            # one mistake can cause
            print("one mistake can cause several lines.")                   # several PROBLEM lines
            print("(From now on, only changes are printed.)")               # e.g. a cable plugged in
        else:                                                               # otherwise:
            print("Everything OK. Next: examples/00_02_encodersbyhand")

    def makeAuto(self):                                                     # builds what autonomous will do
        return None                                                         # nothing moves in this step

    def autonomousInit(self) -> None:                                       # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                                  # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                    # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()            # the part that runs commands
            scheduler.schedule(self.autoCommand)                            # start the auto

    def teleopInit(self) -> None:                                           # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                    # if an auto is still running...
            self.autoCommand.cancel()                                       # stop the auto: the driver takes over

    def testInit(self) -> None:                                             # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()                # stop every command that is running
