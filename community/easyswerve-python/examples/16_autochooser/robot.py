#!/usr/bin/env python3
"""
EXAMPLE: Pick an auto on the dashboard

A SendableChooser shows a drop-down list on the dashboard. The drive team
picks an auto before the match, without changing code.

On the robot: put it in the middle of the practice carpet, facing the long
way. Open Elastic (connected to the robot), pick an auto in the Auto
chooser, then enable in Autonomous.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                    # command-based robot library
from commands2.button import CommandXboxController                  # an Xbox controller with buttons
import wpilib                                                       # WPILib: the library all FRC robots use

import robot_settings                                               # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                   # the swerve drivetrain (4 modules)
from easyswerve import carpet                                       # the practice carpet's size (robot_settings.py)


class MyRobot(commands2.TimedCommandRobot):                         # our robot: runs 50 times a second
    def robotInit(self) -> None:                                    # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                              # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                       # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)               # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)        # left stick drives, right stick turns
        self.chooser = wpilib.SendableChooser()                     # a list of autos shown on the dashboard
        forward = self.driveForward                                 # the function that builds this auto
        self.chooser.setDefaultOption("Drive forward", forward)     # picked unless you choose another
        self.chooser.addOption("Spin", self.spin)                   # another choice: name, function
        self.chooser.addOption("Do nothing", self.doNothing)        # and another
        wpilib.SmartDashboard.putData("Auto", self.chooser)         # show it on the dashboard as "Auto"

        self.autoCommand = None                                     # no auto yet (None = nothing)

    def makeAuto(self):                                             # builds what autonomous will do
        buildAuto = self.chooser.getSelected()                      # the function for the auto that was picked
        return buildAuto()                                          # call it: it builds the auto

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

    def driveForward(self):                                         # builds the "Drive forward" auto
        seconds = carpet.fitSeconds(2.0, forward=0.25)              # 2 s, or less to stay on the carpet
        return self.drivetrain.driveFor(seconds, forward=0.25)      # 2 s forward

    def spin(self):                                                 # builds the "Spin" auto
        return self.drivetrain.driveFor(3.0, turn=0.3)              # 3 s spinning

    def doNothing(self):                                            # builds the "Do nothing" auto
        return None                                                 # None = no auto
