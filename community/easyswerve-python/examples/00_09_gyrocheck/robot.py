#!/usr/bin/env python3
"""
SETUP STEP 9: Check the gyro (IMU)

The navX gyro tells the robot which way it faces. This checks the three things
that go wrong with gyros: still calibrating, drifting while the robot sits
still, and turns that don't add up.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                              # command-based robot library
import wpilib                                                                 # WPILib: the library all FRC robots use
from commands2.button import CommandXboxController                            # an Xbox controller with buttons

import robot_settings                                                         # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                             # the swerve drivetrain (4 modules)


class MyRobot(commands2.TimedCommandRobot):                                   # our robot: runs 50 times a second
    def robotInit(self) -> None:                                              # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                        # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                                 # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                         # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)                  # left stick drives, right stick turns
        reset = self.drivetrain.resetHeading()                                # a command: this way is now forward
        self.controller.start().onTrue(reset)                                 # press Start to run it
        self.driftStartTime = None                                            # when we started measuring drift (None = not yet)
        self.driftStartHeading = 0.0                                          # the heading at that moment
        self.lastPrint = 0.0                                                  # when we last printed
        self.wasCalibrating = None                                            # calibrating last loop? (None = don't know yet)
        self.autoCommand = None                                               # no auto yet (None = nothing)

    def disabledInit(self) -> None:                                           # runs once when the robot is disabled
        self.driftStartTime = None                                            # start a new drift measurement

    def disabledPeriodic(self) -> None:                                       # runs 50 times a second while disabled
        now = wpilib.Timer.getFPGATimestamp()                                 # the time now, in seconds
        heading = self.drivetrain.getHeading()                                # which way the robot faces (degrees)
        if self.drivetrain.gyroCalibrating():                                 # navX still calibrating?
            self.driftStartTime = None                                        # then don't measure drift yet
        elif self.driftStartTime is None:                                     # calibrated, and not measuring yet?
            self.driftStartTime = now                                         # start measuring now,
            self.driftStartHeading = heading                                  # from this heading

        calibrating = self.driftStartTime is None                             # still calibrating?
        if calibrating != self.wasCalibrating:                                # did that just change?
            self.wasCalibrating = calibrating                                 # remember the new state
            if calibrating:
                print("Gyro CALIBRATING: keep the robot still")
            else:                                                             # otherwise:
                print("Gyro calibrated: measuring drift...")
        if calibrating:                                                       # nothing to measure yet
            return

        if now - self.lastPrint < 15.0:                                       # printed less than 15 seconds ago?
            return                                                            # then wait (live numbers are on the dashboard)
        self.lastPrint = now

        minutes = (now - self.driftStartTime) / 60.0                          # how long we've measured
        drift = heading - self.driftStartHeading                              # how far the heading crept
        perMinute = drift / minutes if minutes > 0 else 0.0                   # drift per minute
        print(f"Gyro drift: {perMinute:+.2f} deg/min (under 1 is fine)")      # one line
        board = wpilib.SmartDashboard                                         # the dashboard
        board.putNumber("Gyro/Drift (deg per minute)", perMinute)             # show it there

    def robotPeriodic(self) -> None:                                          # runs 50 times a second, in every mode
        board = wpilib.SmartDashboard                                         # the dashboard
        dt = self.drivetrain                                                  # a shorter name for the drivetrain
        board.putBoolean("Gyro/Calibrating", dt.gyroCalibrating())            # True while calibrating
        board.putNumber("Gyro/Heading (deg)", dt.getHeading())                # which way it faces
        board.putNumber("Gyro/Turn rate (deg per s)", dt.getTurnRate())       # how fast it turns

    def makeAuto(self):                                                       # builds what autonomous will do
        return commands2.SequentialCommandGroup(                              # turn each way, then back to the start:
            commands2.PrintCommand("Turning test: mark where it faces now"),
            self.drivetrain.turnTo(90),                                       # face left
            self.drivetrain.turnTo(180),                                      # face backwards
            self.drivetrain.turnTo(-90),                                      # face right
            self.drivetrain.turnTo(0),                                        # back to the start
            commands2.PrintCommand("Turning test done: facing the mark?"),    # if the gyro is right
        )

    def autonomousInit(self) -> None:                                         # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                                    # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                      # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()              # the part that runs commands
            scheduler.schedule(self.autoCommand)                              # start the auto

    def teleopInit(self) -> None:                                             # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                      # if an auto is still running...
            self.autoCommand.cancel()                                         # stop the auto: the driver takes over

    def testInit(self) -> None:                                               # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()                  # stop every command that is running
