#!/usr/bin/env python3
"""
SETUP STEP 4: Zero the steering encoders

Teach the robot which way "straight ahead" is for each wheel. Do this once on
a new robot, and again whenever a module or encoder is taken apart, or after
changing encoder_reversed or encoder_dio (so do 00_03 first).

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import math                                                           # math: sin, cos, pi, ...

import commands2                                                      # command-based robot library
import wpilib                                                         # WPILib: the library all FRC robots use
from commands2.button import CommandXboxController                    # an Xbox controller with buttons

import robot_settings                                                 # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                     # the swerve drivetrain (4 modules)


class MyRobot(commands2.TimedCommandRobot):                           # our robot: runs 50 times a second
    def robotInit(self) -> None:                                      # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                         # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                 # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)          # left stick drives, right stick turns
        zero = self.drivetrain.zeroSteeringCommand()                  # a command: save the wheel angles as zero
        self.controller.back().onTrue(zero)                           # press Back (DISABLED, hex keys in)
        self.lastPrint = 0.0                                          # when we last printed (seconds)
        self.lastAngles = []                                          # the angles we last printed (none yet)
        self.autoCommand = None                                       # no auto yet (None = nothing)

    def disabledPeriodic(self) -> None:                               # runs 50 times a second while disabled
        now = wpilib.Timer.getFPGATimestamp()                         # the time now, in seconds
        if now - self.lastPrint < 0.5:                                # checked less than half a second ago?
            return                                                    # then wait
        degrees = []                                                  # each wheel's angle (degrees)
        for name, module in self.drivetrain.modules:                  # repeat for each of the 4 modules
            rad = module.getState().angle.radians()                   # which way the wheel points (radians)
            degrees.append(math.degrees(rad))                         # the same, in degrees (0 = straight ahead)
        pairs = zip(degrees, self.lastAngles)                         # (now, last printed) for each wheel
        moved = [abs(d - old) > 1.0 for d, old in pairs]              # changed more than 1 degree?
        if self.lastAngles and not any(moved):                        # printed before, and nothing moved?
            return                                                    # then don't print the same numbers again
        self.lastPrint = now                                          # remember when we printed
        self.lastAngles = degrees                                     # and what
        names = [name for name, _ in self.drivetrain.modules]         # ["FL", "FR", "RL", "RR"]
        angles = [f"{n} {d:6.1f}" for n, d in zip(names, degrees)]    # e.g. "FL   45.0"
        print("Wheel angles: " + "   ".join(angles))                  # after zeroing: FL 45, FR -45, RL 135, RR -135

    def makeAuto(self):                                               # builds what autonomous will do
        return self.drivetrain.driveFor(1.5, forward=0.15)            # check: drive slowly; all wheels straight?

    def autonomousInit(self) -> None:                                 # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                            # build the auto (makeAuto, above)
        if self.autoCommand is not None:                              # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()      # the part that runs commands
            scheduler.schedule(self.autoCommand)                      # start the auto

    def teleopInit(self) -> None:                                     # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                              # if an auto is still running...
            self.autoCommand.cancel()                                 # stop the auto: the driver takes over

    def testInit(self) -> None:                                       # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()          # stop every command that is running
