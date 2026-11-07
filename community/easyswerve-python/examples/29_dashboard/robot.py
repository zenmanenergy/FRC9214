#!/usr/bin/env python3
"""
EXAMPLE: Show numbers on the dashboard

robotPeriodic runs 50 times a second in every mode. SmartDashboard puts
values on the dashboard, where you can watch them live.

On the robot: put it in the middle of the practice carpet, facing the long
way. Open Elastic (connected to the robot), enable in Autonomous, and add the My Robot values to a tab.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                          # command-based robot library
from commands2.button import CommandXboxController                        # an Xbox controller with buttons
import wpilib                                                             # WPILib: the library all FRC robots use

import robot_settings                                                     # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                         # the swerve drivetrain (4 modules)
from easyswerve import carpet                                             # the practice carpet's size (robot_settings.py)


class MyRobot(commands2.TimedCommandRobot):                               # our robot: runs 50 times a second
    def robotInit(self) -> None:                                          # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                    # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                             # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                     # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)              # left stick drives, right stick turns

        self.autoCommand = None                                           # no auto yet (None = nothing)

    def makeAuto(self):                                                   # builds what autonomous will do
        seconds = carpet.fitSeconds(3.0, forward=0.3, turn=0.15)          # 3 s, or less to stay on the carpet
        return self.drivetrain.driveFor(seconds, forward=0.3, turn=0.15)  # drive in a curve for 3 s

    def autonomousInit(self) -> None:                                     # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                                # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                  # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()          # the part that runs commands
            scheduler.schedule(self.autoCommand)                          # start the auto

    def teleopInit(self) -> None:                                         # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                  # if an auto is still running...
            self.autoCommand.cancel()                                     # stop the auto: the driver takes over

    def testInit(self) -> None:                                           # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()              # stop every command that is running

    def robotPeriodic(self) -> None:                                      # runs 50 times a second, in every mode
        pose = self.drivetrain.getPose()                                  # where the robot thinks it is (x, y, heading)
        board = wpilib.SmartDashboard                                     # a shorter name for the dashboard
        board.putNumber("My Robot/X (m)", pose.X())                       # show x (meters) on the dashboard
        board.putNumber("My Robot/Y (m)", pose.Y())                       # and y
        heading = self.drivetrain.getHeading()                            # which way it faces (degrees)
        board.putNumber("My Robot/Heading (deg)", heading)                # show it
        enabled = wpilib.DriverStation.isEnabled()                        # True while the robot is enabled
        board.putBoolean("My Robot/Enabled", enabled)                     # a True/False light
