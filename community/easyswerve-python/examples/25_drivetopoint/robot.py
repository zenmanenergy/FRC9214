#!/usr/bin/env python3
"""
EXAMPLE: Drive to a spot by itself

Hold A and the robot drives itself to a spot on the field, steering around
obstacles. Let go and you're driving again. Spots are in meters from the
blue alliance corner.

On the robot: plug a controller into the Driver Station laptop,
and enable in Teleoperated. Drive somewhere, then hold A.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                    # command-based robot library
from commands2.button import CommandXboxController                  # an Xbox controller with buttons

import robot_settings                                               # load robot_settings.py (your robot's settings)
from easyswerve import Autos, Drivetrain                            # drivetrain, and PathPlanner autos


class MyRobot(commands2.TimedCommandRobot):                         # our robot: runs 50 times a second
    def robotInit(self) -> None:                                    # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                              # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                       # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)               # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)        # left stick drives, right stick turns

        self.autos = Autos(self.drivetrain)                         # autos drawn in the PathPlanner app
        goTo = self.autos.driveTo(2.0, 4.0, 0)                      # a command: drive to x=2, y=4, facing 0
        self.controller.a().whileTrue(goTo)                         # hold A: drive there by itself

        self.autoCommand = None                                     # no auto yet (None = nothing)

    def makeAuto(self):                                             # builds what autonomous will do
        return None                                                 # None = no auto

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
