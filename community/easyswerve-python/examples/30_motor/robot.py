#!/usr/bin/env python3
"""
EXAMPLE: Spin a motor with a button

The simplest mechanism: one motor on a SPARK MAX. Hold A to spin it one way,
B the other way. Set MOTOR_CAN_ID to your motor's CAN ID first, and only
use this once that motor is wired up and free to spin.

On the robot: plug a controller into the Driver Station laptop,
and enable in Teleoperated. Hold A or B and watch the motor.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                    # command-based robot library
from commands2.button import CommandXboxController                  # an Xbox controller with buttons
from rev import SparkMax                                            # REV's library: SPARK MAX motor controllers

import robot_settings                                               # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                   # the swerve drivetrain (4 modules)


MOTOR_CAN_ID = 20                                                   # change to your motor's CAN ID


class MyRobot(commands2.TimedCommandRobot):                         # our robot: runs 50 times a second
    def robotInit(self) -> None:                                    # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                              # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                       # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)               # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)        # left stick drives, right stick turns

        kind = SparkMax.MotorType.kBrushless                        # NEO motors are brushless
        self.motor = SparkMax(MOTOR_CAN_ID, kind)                   # the SPARK MAX with that CAN ID
        self.controller.a().whileTrue(                              # hold A to run this command:
            commands2.StartEndCommand(                              # one thing at the start, another at the end:
                lambda: self.motor.set(0.5),                        # start (A pressed): 50 % power forward
                lambda: self.motor.set(0),                          # end (A let go): stop
            )
        )
        self.controller.b().whileTrue(                              # hold B: the same, backward
            commands2.StartEndCommand(
                lambda: self.motor.set(-0.5),                       # negative power = backward
                lambda: self.motor.set(0),
            )
        )

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
