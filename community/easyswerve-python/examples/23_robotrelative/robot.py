#!/usr/bin/env python3
"""
EXAMPLE: Robot-relative driving

Normally pushing forward drives away from the driver, whichever way the
robot faces (field-relative). Robot-relative means forward is wherever the
robot's front points, like a remote-control car.

On the robot: plug a controller into the Driver Station laptop,
and enable in Teleoperated.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                         # command-based robot library
from commands2.button import CommandXboxController                       # an Xbox controller with buttons

import robot_settings                                                    # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                        # the swerve drivetrain (4 modules)


class MyRobot(commands2.TimedCommandRobot):                              # our robot: runs 50 times a second
    def robotInit(self) -> None:                                         # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                   # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                            # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                    # the driver's Xbox controller
        pad = self.controller                                            # a shorter name for the controller
        self.drivetrain.driveWithController(pad, fieldRelative=False)    # sticks move it the way IT faces

        self.autoCommand = None                                          # no auto yet (None = nothing)

    def makeAuto(self):                                                  # builds what autonomous will do
        return None                                                      # None = no auto

    def autonomousInit(self) -> None:                                    # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                               # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                 # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()         # the part that runs commands
            scheduler.schedule(self.autoCommand)                         # start the auto

    def teleopInit(self) -> None:                                        # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                 # if an auto is still running...
            self.autoCommand.cancel()                                    # stop the auto: the driver takes over

    def testInit(self) -> None:                                          # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()             # stop every command that is running
