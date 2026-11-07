#!/usr/bin/env python3
"""
EXAMPLE: Back and forth, again and again

.repeatedly() runs a command over and over. .withTimeout() stops it after
some time, so this goes back and forth for 8 seconds.

On the robot: put it in the middle of the practice carpet, facing the long
way, and enable in Autonomous. Its moves fit the carpet size in
robot_settings.py (CARPET_LENGTH_M and CARPET_WIDTH_M).

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                    # command-based robot library
from commands2.button import CommandXboxController                  # an Xbox controller with buttons

import robot_settings                                               # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                   # the swerve drivetrain (4 modules)
from easyswerve import carpet                                       # the practice carpet's size (robot_settings.py)


class MyRobot(commands2.TimedCommandRobot):                         # our robot: runs 50 times a second
    def robotInit(self) -> None:                                    # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                              # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                       # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)               # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)        # left stick drives, right stick turns

        self.autoCommand = None                                     # no auto yet (None = nothing)

    def makeAuto(self):                                             # builds what autonomous will do
        seconds = carpet.fitSeconds(1.0, forward=0.25)              # 1 s, or less to stay on the carpet
        backAndForth = commands2.SequentialCommandGroup(            # one trip, saved in a variable:
            self.drivetrain.driveFor(seconds, forward=0.25),        # forward for 1 s...
            self.drivetrain.driveFor(seconds, forward=-0.25),       # ...then back for 1 s
        )
        return backAndForth.repeatedly().withTimeout(8.0)           # repeat the trip, stop after 8 s

    def autonomousInit(self) -> None:                               # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                          # build the auto (makeAuto, above)
        if self.autoCommand is not None:                            # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()    # the part that runs commands
            scheduler.schedule(self.autoCommand)                    # start the auto

    def teleopInit(self) -> None:                                   # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                            # if an auto is still running...
            self.autoCommand.cancel()                               # stop the auto: the driver takes over

    def testInit(self) -> None:                                     # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()        # stop every command that is running
