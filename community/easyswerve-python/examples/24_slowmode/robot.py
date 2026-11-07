#!/usr/bin/env python3
"""
EXAMPLE: Slow mode

Hold the left trigger to drive at 30 % speed, for careful lining up. This
writes its own driving command instead of using driveWithController.

On the robot: plug a controller into the Driver Station laptop,
and enable in Teleoperated. Drive, then hold the left trigger.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                     # command-based robot library
from commands2.button import CommandXboxController                   # an Xbox controller with buttons
import wpimath                                                       # WPILib's math tools

import robot_settings                                                # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                    # the swerve drivetrain (4 modules)


class MyRobot(commands2.TimedCommandRobot):                          # our robot: runs 50 times a second
    def robotInit(self) -> None:                                     # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                               # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                        # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                # the driver's Xbox controller

        self.drivetrain.setDefaultCommand(                           # what the drivetrain does when nothing else runs:
            commands2.RunCommand(self.driveSlow, self.drivetrain)    # call driveSlow every loop
        )

        self.autoCommand = None                                      # no auto yet (None = nothing)

    def makeAuto(self):                                              # builds what autonomous will do
        return None                                                  # None = no auto

    def autonomousInit(self) -> None:                                # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                           # build the auto (makeAuto, above)
        if self.autoCommand is not None:                             # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()     # the part that runs commands
            scheduler.schedule(self.autoCommand)                     # start the auto

    def teleopInit(self) -> None:                                    # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                             # if an auto is still running...
            self.autoCommand.cancel()                                # stop the auto: the driver takes over

    def testInit(self) -> None:                                      # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()         # stop every command that is running

    def driveSlow(self) -> None:                                     # runs 50 times a second while driving
        pad = self.controller                                        # a shorter name for the controller
        slow = pad.getLeftTriggerAxis() > 0.5                        # left trigger pulled more than halfway?
        speed = 0.3 if slow else 1.0                                 # 30 % speed if so, otherwise full speed
        forward = -wpimath.applyDeadband(pad.getLeftY(), 0.1)        # stick up = forward (ignore tiny moves)
        left = -wpimath.applyDeadband(pad.getLeftX(), 0.1)           # stick left = move left
        turn = -wpimath.applyDeadband(pad.getRightX(), 0.1)          # right stick left = turn left

        f, l, t = forward * speed, left * speed, turn * speed        # slow all three down
        self.drivetrain.drive(f, l, t, True)                         # drive (True = field-relative)
