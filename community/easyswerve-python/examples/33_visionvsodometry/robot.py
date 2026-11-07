#!/usr/bin/env python3
"""
EXAMPLE: Wheels only vs. wheels + cameras

The robot keeps track of where it is in two ways here, side by side:
  - wheels only (odometry): wheel encoders and the gyro
  - wheels + cameras: the drivetrain's real position, corrected by AprilTags
Every 2 seconds while it drives, it prints both. When the wheels slip, the
wheels-only position drifts away; the camera-corrected one stays right.

Setup first: VISION_ENABLED = True in robot_settings.py (with measured
cameras, measured=True), remove the # before photonlibpy in pyproject.toml,
run py -m robotpy sync, then deploy.

On the robot: put it about 2 m in front of an AprilTag, facing it, with
2 m of clear floor behind it. Enable in Autonomous: it drives back and forth
for 12 seconds. Watch the console, then measure where the robot really is
with a tape measure and see which position was closer.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                      # command-based robot library
import wpilib                                                         # WPILib: the library all FRC robots use
from commands2.button import CommandXboxController                    # an Xbox controller with buttons
from wpimath.kinematics import SwerveDrive4Odometry

import robot_settings                                                 # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                     # the swerve drivetrain (4 modules)
from easyswerve import carpet                                         # the practice carpet's size (robot_settings.py)


class MyRobot(commands2.TimedCommandRobot):                           # our robot: runs 50 times a second
    def robotInit(self) -> None:                                      # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                         # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                 # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)          # left stick drives, right stick turns
        self.wheelsOnly = SwerveDrive4Odometry(                       # our own wheels-only tracker (no cameras):
            self.drivetrain.kinematics,                               # the robot's shape (where the wheels are)
            self.drivetrain.gyroRotation(),                           # which way it faces
            self.drivetrain.modulePositions(),                        # how far each wheel rolled, which way
        )
        self.lastPrint = 0.0                                          # when we last printed
        self.autoCommand = None                                       # no auto yet (None = nothing)

    def robotPeriodic(self) -> None:                                  # runs 50 times a second, in every mode
        heading = self.drivetrain.gyroRotation()                      # the gyro, now
        wheels = self.drivetrain.modulePositions()                    # the wheels, now
        self.wheelsOnly.update(heading, wheels)                       # update the wheels-only tracker every loop

        now = wpilib.Timer.getFPGATimestamp()                         # the time now, in seconds
        if not wpilib.DriverStation.isEnabled():                      # only print while the robot moves
            return
        if now - self.lastPrint < 2.0:                                # printed less than 2 seconds ago?
            return                                                    # then wait
        self.lastPrint = now
        self.printBoth()                                              # one line: both positions

    def disabledInit(self) -> None:                                   # runs once when the robot is disabled
        if self.lastPrint > 0:                                        # if it was driving before...
            print("Final:")
            self.printBoth()                                          # ...print where it ended up

    def printBoth(self) -> None:                                      # print both positions on one line
        wheels = self.wheelsOnly.getPose()                            # position by wheels only
        fused = self.drivetrain.getPose()                             # position by wheels + cameras
        apart = wheels.translation().distance(fused.translation())    # how far apart (m)
        w = f"wheels only ({wheels.X():.2f}, {wheels.Y():.2f})"       # (x, y) in meters
        f = f"wheels + cameras ({fused.X():.2f}, {fused.Y():.2f})"
        print(f"{w}  {f}  {apart:.2f} m apart")

    def makeAuto(self):                                               # builds what autonomous will do
        seconds = carpet.fitSeconds(1.5, forward=0.3)                 # 1.5 s, or less to stay on the carpet
        backAndForth = commands2.SequentialCommandGroup(              # one trip:
            self.drivetrain.driveFor(seconds, forward=-0.3),          # back away from the tag...
            self.drivetrain.driveFor(seconds, forward=0.3),           # ...and come back
        )
        return commands2.SequentialCommandGroup(                      # these run one after another:
            commands2.InstantCommand(self.startTogether),             # 1. both trackers start together
            backAndForth.repeatedly().withTimeout(12.0),              # 2. repeat the trip for 12 s
        )

    def startTogether(self) -> None:                                  # put the wheels-only tracker where the robot is now
        self.wheelsOnly.resetPosition(                                # start it from:
            self.drivetrain.gyroRotation(),                           # the gyro now
            self.drivetrain.modulePositions(),                        # the wheels now
            self.drivetrain.getPose(),                                # and the drivetrain's position
        )

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
