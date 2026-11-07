#!/usr/bin/env python3
"""
EXAMPLE: Basic robot (the starting robot.py)

Driving with a controller, two buttons, and autos from the PathPlanner app.
Copy this back over robot.py to start over.

On the robot: in the dashboard (Elastic), pick "Test S-Curve" in the Auto
chooser. Put the robot at one end of the practice carpet, facing the long
way, centered, back bumper about 20 cm from the edge, then enable in
Autonomous. The robot drives a gentle 1.2 m curve.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                    # command-based robot library
from commands2.button import CommandXboxController                  # an Xbox controller with buttons

import robot_settings                                               # load robot_settings.py (your robot's settings)
from easyswerve import Autos, Drivetrain                            # drivetrain, and PathPlanner autos

STARTING_ROBOT = True                                               # the deploy tests check the sticks and buttons of this robot.py


class MyRobot(commands2.TimedCommandRobot):                         # our robot: runs 50 times a second
    def robotInit(self) -> None:                                    # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                              # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                       # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)               # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)        # left stick drives, right stick turns
        lock = self.drivetrain.lockWheels()                         # a command: wheels in an X (hard to push)
        self.controller.rightBumper().whileTrue(lock)               # runs while the right bumper is held
        reset = self.drivetrain.resetHeading()                      # a command: this way is now forward
        self.controller.start().onTrue(reset)                       # press Start to run it
        self.autos = Autos(self.drivetrain)                         # autos drawn in PathPlanner (deploy/pathplanner/)
        hello = commands2.PrintCommand("Hello from auto!")          # a command that prints a message
        self.autos.addCommand("printHello", hello)                  # autos can run it by this name

        self.autoCommand = None                                     # no auto yet (None = nothing)

    def makeAuto(self):                                             # builds what autonomous will do
        return self.autos.selected()                                # the auto picked in the dashboard's "Auto" chooser

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
