#!/usr/bin/env python3
"""
SETUP STEP 10: Measure the wheel diameter

The drive encoders count wheel turns. The code turns that into meters using
WHEEL_DIAMETER_M in robot_settings.py. If that number is wrong (treads wear
down), every distance and every path is off. This drives a test distance;
you measure how far it really went, and the robot works out the right number.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                           # command-based robot library
import wpilib                                                              # WPILib: the library all FRC robots use
from commands2.button import CommandXboxController                         # an Xbox controller with buttons

import robot_settings                                                      # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                          # the swerve drivetrain (4 modules)
from easyswerve import carpet                                              # the practice carpet's size (robot_settings.py)

TEST_DISTANCE_M = carpet.fitMeters(2.0, carpet.END_AHEAD_M)                # 2 m, or less to fit the carpet (from one end)
BOX = "Wheel Check/Measured distance (m)"                                  # the name of the dashboard box


class MyRobot(commands2.TimedCommandRobot):                                # our robot: runs 50 times a second
    def robotInit(self) -> None:                                           # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                     # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                              # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                      # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)               # left stick drives, right stick turns
        wpilib.SmartDashboard.putNumber(BOX, 0.0)                          # a dashboard box to type the distance in
        self.startDistances = None                                         # each wheel's distance at the start
        self.encoderDistance = None                                        # how far the encoders say it went
        self.autoCommand = None                                            # no auto yet (None = nothing)

    def makeAuto(self):                                                    # builds what autonomous will do
        return commands2.SequentialCommandGroup(                           # these run one after another:
            commands2.InstantCommand(self.startMeasuring),                 # 1. note the wheel distances
            self.drivetrain.driveDistance(TEST_DISTANCE_M, speed=0.15),    # 2. drive (slow: no slipping)
            commands2.WaitCommand(0.5),                                    # 3. let it come to a full stop
            commands2.InstantCommand(self.stopMeasuring),                  # 4. how far did they roll?
        )

    def startMeasuring(self) -> None:                                      # at the start of the test drive
        self.startDistances = self.drivetrain.wheelDistances()             # each wheel's distance now
        print("Test drive: put a tape mark at the front bumper now")

    def stopMeasuring(self) -> None:                                       # at the end of the test drive
        end = self.drivetrain.wheelDistances()                             # each wheel's distance now
        pairs = zip(self.startDistances, end)                              # (start, end) for each wheel
        rolled = [abs(e - s) for s, e in pairs]                            # how far each wheel rolled
        self.encoderDistance = sum(rolled) / len(rolled)                   # the average of the 4
        print(f"Encoders say the robot drove {self.encoderDistance:.3f} m.")
        off = (self.encoderDistance - TEST_DISTANCE_M) * 100               # how far from the distance asked for (cm)
        print(f"Asked for {TEST_DISTANCE_M} m: it stopped {off:+.1f} cm off (by its encoders).")
        if abs(off) > 3:                                                   # more than 3 cm off?
            print("  That's a lot: is something rubbing, or the battery low?")
        print("Measure from the tape mark to the front bumper")
        print(f'and type it into the dashboard box "{BOX}".')

    def disabledPeriodic(self) -> None:                                    # runs 50 times a second while disabled
        if self.encoderDistance is None:                                   # no test drive yet?
            return
        measured = wpilib.SmartDashboard.getNumber(BOX, 0.0)               # what was typed in the box
        if measured <= 0:                                                  # nothing typed in yet?
            return
        ratio = measured / self.encoderDistance                            # real distance / encoder distance
        correct = robot_settings.WHEEL_DIAMETER_M * ratio                  # the real diameter: same ratio
        board = wpilib.SmartDashboard                                      # the dashboard
        board.putNumber("Wheel Check/Set WHEEL_DIAMETER_M to", correct)    # show it there

        if measured != getattr(self, "lastMeasured", None):                # a new number typed in?
            self.lastMeasured = measured                                   # remember it (print only once)
            print(f"Measured {measured:.3f} m, encoders {self.encoderDistance:.3f} m.")
            print(f"Set WHEEL_DIAMETER_M = {correct:.4f} in robot_settings.py")

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
