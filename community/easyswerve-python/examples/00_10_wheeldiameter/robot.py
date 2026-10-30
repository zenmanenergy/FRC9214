#!/usr/bin/env python3
"""
SETUP STEP 10: Measure the wheel diameter

The drive encoders count wheel turns. The code turns that into meters using
WHEEL_DIAMETER_M in robot_settings.py. If that number is wrong (treads wear
down), every distance and every path is off. This drives a test distance;
you measure how far it really went, and the robot works out the right number.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy
"""

import commands2
import wpilib
from commands2.button import CommandXboxController

import robot_settings
from easyswerve import Drivetrain

TEST_DISTANCE_M = 2.0  # how far the test drive goes, by the encoders


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        # A box on the dashboard to type the measured distance into
        wpilib.SmartDashboard.putNumber("Wheel Check/Measured distance (m)", 0.0)

        self.startDistances = None
        self.encoderDistance = None
        self.autoCommand = None

    # ---- the test drive -----------------------------------------------------------------

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        return commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.startMeasuring),
            # Slow, so the wheels don't slip
            self.drivetrain.driveDistance(TEST_DISTANCE_M, speed=0.15),
            commands2.WaitCommand(0.5),  # let the robot come to a full stop
            commands2.InstantCommand(self.stopMeasuring),
        )

    def startMeasuring(self) -> None:
        self.startDistances = self.drivetrain.wheelDistances()
        print("Test drive: put a tape mark at the front bumper now")

    def stopMeasuring(self) -> None:
        end = self.drivetrain.wheelDistances()
        # How far each wheel rolled, by its encoder, averaged over 4 wheels
        rolled = [abs(e - s) for s, e in zip(self.startDistances, end)]
        self.encoderDistance = sum(rolled) / len(rolled)
        print(f"Encoders say the robot drove {self.encoderDistance:.3f} m.")
        print("Measure from the tape mark to the front bumper, and type it into")
        print('the dashboard box "Wheel Check/Measured distance (m)".')

    # ---- the math ------------------------------------------------------------------------

    def disabledPeriodic(self) -> None:
        if self.encoderDistance is None:
            return  # no test drive yet
        measured = wpilib.SmartDashboard.getNumber("Wheel Check/Measured distance (m)", 0.0)
        if measured <= 0:
            return  # nothing typed in yet

        # Each wheel turn rolls pi x diameter. The encoders used the diameter
        # in robot_settings.py; the real diameter is in the same ratio as the
        # real distance to the encoder distance.
        correct = robot_settings.WHEEL_DIAMETER_M * measured / self.encoderDistance
        wpilib.SmartDashboard.putNumber("Wheel Check/Set WHEEL_DIAMETER_M to", correct)

        if measured != getattr(self, "lastMeasured", None):
            self.lastMeasured = measured
            print(
                f"Measured {measured:.3f} m, encoders {self.encoderDistance:.3f} m. "
                f"Set WHEEL_DIAMETER_M = {correct:.4f} in robot_settings.py"
            )

    def autonomousInit(self) -> None:
        self.autoCommand = self.makeAuto()
        if self.autoCommand is not None:
            commands2.CommandScheduler.getInstance().schedule(self.autoCommand)

    def teleopInit(self) -> None:
        # Stop the auto when the driver takes over
        if self.autoCommand is not None:
            self.autoCommand.cancel()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()
