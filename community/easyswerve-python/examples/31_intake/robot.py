#!/usr/bin/env python3
"""
EXAMPLE: An intake mechanism

A mechanism as a subsystem (a class), with commands it can do. The buttons
use them in teleop, and the auto drives forward while the intake runs.

On the robot: put it in the middle of the practice carpet, facing the long
way, enable in Autonomous, and watch the console: intake
on, then off when the drive ends.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                           # command-based robot library
from commands2.button import CommandXboxController                         # an Xbox controller with buttons
from rev import SparkMax                                                   # REV's library: SPARK MAX motor controllers

import robot_settings                                                      # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                          # the swerve drivetrain (4 modules)
from easyswerve import carpet                                              # the practice carpet's size (robot_settings.py)


INTAKE_CAN_ID = 20                                                         # change to your intake's CAN ID
INTAKE_SPEED = 0.6                                                         # 0 to 1 (fraction of full power)


class Intake(commands2.Subsystem):                                         # a one-motor intake: a subsystem of its own
    def __init__(self) -> None:                                            # runs once, when Intake() is made
        super().__init__()                                                 # set up the Subsystem part first
        kind = SparkMax.MotorType.kBrushless                               # NEO motors are brushless
        self.motor = SparkMax(INTAKE_CAN_ID, kind)                         # its SPARK MAX

    def runIn(self) -> commands2.Command:                                  # a command: pull in while it runs
        return commands2.StartEndCommand(self.start, self.stop, self)      # start() at the start, stop() at the end

    def runOut(self) -> commands2.Command:                                 # a command: spit out while it runs
        return commands2.StartEndCommand(self.reverse, self.stop, self)    # reverse() at the start, stop() at the end

    def start(self) -> None:                                               # motor on, pulling in
        print("intake on")                                                 # shows in the console
        self.motor.set(INTAKE_SPEED)                                       # set the power

    def reverse(self) -> None:                                             # motor on, pushing out
        print("intake reversing")
        self.motor.set(-INTAKE_SPEED)                                      # negative power = the other way

    def stop(self) -> None:                                                # motor off
        print("intake off")
        self.motor.set(0)                                                  # 0 power = stop


class MyRobot(commands2.TimedCommandRobot):                                # our robot: runs 50 times a second
    def robotInit(self) -> None:                                           # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                     # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                              # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                      # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)               # left stick drives, right stick turns

        self.intake = Intake()                                             # make the intake (the class above)
        self.controller.a().whileTrue(self.intake.runIn())                 # hold A: pull in
        self.controller.b().whileTrue(self.intake.runOut())                # hold B: spit out

        self.autoCommand = None                                            # no auto yet (None = nothing)

    def makeAuto(self):                                                    # builds what autonomous will do
        seconds = carpet.fitSeconds(2.0, forward=0.2)                      # 2 s, or less to stay on the carpet
        return commands2.ParallelDeadlineGroup(                            # run these at the same time, until the FIRST ends:
            self.drivetrain.driveFor(seconds, forward=0.2),                # the deadline: drive forward 2 s
            self.intake.runIn(),                                           # runs alongside, stops when the drive ends
        )

    def autonomousInit(self) -> None:                                      # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                                 # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                   # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()           # the part that runs commands
            scheduler.schedule(self.autoCommand)                           # start the auto

    def teleopInit(self) -> None:                                          # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                   # if an auto is still running...
            self.autoCommand.cancel()                                      # stop the auto: the driver takes over

    def testInit(self) -> None:                                            # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()               # stop every command that is running
