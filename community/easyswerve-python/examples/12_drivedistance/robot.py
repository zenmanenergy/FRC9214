#!/usr/bin/env python3
"""
EXAMPLE: Drive a distance

driveFor uses time; driveDistance uses the wheel encoders to drive an exact
distance, however fast it goes. Negative meters drives backward.

On the robot: put it in the middle of the practice carpet, facing the long
way, and enable in Autonomous. The robot stops 2 meters from where it
started, or less if the carpet is too short (carpet.fitMeters).

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
        meters = carpet.fitMeters(2.0)                              # 2 m, or less to stay on the carpet
        return self.drivetrain.driveDistance(meters, speed=0.3)     # drive exactly that far (wheel encoders), speed 0.3

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
