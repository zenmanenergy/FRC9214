#!/usr/bin/env python3
"""
EXAMPLE: Wheels only vs. wheels + cameras

The robot keeps track of where it is in two ways here, side by side:
  - wheels only (odometry): wheel encoders and the gyro
  - wheels + cameras: the drivetrain's real position, corrected by AprilTags
Every second it prints both. When the wheels slip, the wheels-only position
drifts away; the camera-corrected one stays right.

Setup first: VISION_ENABLED = True in robot_settings.py, and in the simulator
also remove the # before photonlibpy in pyproject.toml, then py -m robotpy sync.
To make the wheels slip in the simulator, set SIM_WHEEL_SLIP = 0.05 at the top
of easyswerve/sim/physics.py.

Try it in the simulator: click Autonomous and watch the terminal. "truth" in
the simulator's status line is where the robot really is.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import commands2
import wpilib
from commands2.button import CommandXboxController
from wpimath.kinematics import SwerveDrive4Odometry

import robot_settings
from easyswerve import Drivetrain


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        # Our own wheels-only position tracker, fed the same wheel and gyro
        # readings as the drivetrain, but never corrected by the cameras
        self.wheelsOnly = SwerveDrive4Odometry(
            self.drivetrain.kinematics,
            self.drivetrain.gyroRotation(),
            self.drivetrain.modulePositions(),
        )

        self.lastPrint = 0.0
        self.autoCommand = None

    def robotPeriodic(self) -> None:
        """Runs 50 times a second in every mode."""
        # Update the wheels-only tracker every loop
        self.wheelsOnly.update(self.drivetrain.gyroRotation(), self.drivetrain.modulePositions())

        now = wpilib.Timer.getFPGATimestamp()
        if now - self.lastPrint < 1.0:
            return
        self.lastPrint = now

        wheels = self.wheelsOnly.getPose()
        fused = self.drivetrain.getPose()
        apart = wheels.translation().distance(fused.translation())
        print(
            f"wheels only x={wheels.X():5.2f} y={wheels.Y():5.2f} | "
            f"wheels + cameras x={fused.X():5.2f} y={fused.Y():5.2f} | "
            f"{apart:.2f} m apart"
        )

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        backAndForth = commands2.SequentialCommandGroup(
            self.drivetrain.driveFor(1.5, forward=-0.3),  # back away from the tag...
            self.drivetrain.driveFor(1.5, forward=0.3),  # ...and come back
        )
        return commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.startInFrontOfTag),
            backAndForth.repeatedly().withTimeout(12.0),
        )

    def startInFrontOfTag(self) -> None:
        # In the simulator, put the robot 2 m in front of the first tag, facing
        # it. On the real robot, place it where the front camera sees a tag.
        if wpilib.RobotBase.isSimulation():
            spot = self.drivetrain.vision.poseInFrontOfTag(distanceM=2.0)
            if spot is not None:
                self.drivetrain.resetOdometry(spot)
        # Both trackers start from the same place
        self.wheelsOnly.resetPosition(
            self.drivetrain.gyroRotation(),
            self.drivetrain.modulePositions(),
            self.drivetrain.getPose(),
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
