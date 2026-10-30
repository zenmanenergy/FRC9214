#!/usr/bin/env python3
"""
EXAMPLE: Drive up to an AprilTag

Hold A and the robot drives itself to a spot 1 meter in front of an AprilTag,
facing it. The tag's position comes from the field map; the robot's position
comes from the wheels, gyro and cameras together. Good for lining up to score.

Setup first: VISION_ENABLED = True in robot_settings.py, and in the simulator
also remove the # before photonlibpy in pyproject.toml, then py -m robotpy sync.

Try it in the simulator: click Autonomous. The robot starts 3 m away from a
tag, off to the side, and drives up to it.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import commands2
import wpilib
from commands2.button import CommandXboxController
from wpimath.geometry import Rotation2d, Transform2d

import robot_settings
from easyswerve import Autos, Drivetrain

TAG_ID = None  # which tag to drive to; None = the first tag on the field
STOP_DISTANCE_M = 1.0  # how far in front of the tag to stop


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)
        self.vision = self.drivetrain.vision
        self.autos = Autos(self.drivetrain)

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        # Where to stop: 1 m in front of the tag, facing it (None if vision is off)
        self.goal = self.vision.poseInFrontOfTag(TAG_ID, STOP_DISTANCE_M)
        if self.goal is None:
            print("Vision is off: set VISION_ENABLED = True in robot_settings.py")
        else:
            print(f"Goal: x={self.goal.X():.2f} y={self.goal.Y():.2f} "
                  f"facing {self.goal.rotation().degrees():.0f} degrees")

            # Hold A: drive there by itself
            self.controller.a().whileTrue(self.driveToGoal())

        self.autoCommand = None

    def driveToGoal(self) -> commands2.Command:
        return self.autos.driveTo(
            self.goal.X(), self.goal.Y(), self.goal.rotation().degrees(), maxSpeedMps=1.5
        )

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        if self.goal is None:
            return None
        return commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.startAwayFromTag),
            self.driveToGoal(),
            commands2.PrintCommand("Arrived in front of the tag"),
        )

    def startAwayFromTag(self) -> None:
        # In the simulator, start 2 m further back and 1 m to the side of the
        # goal. On the real robot, place it anywhere the cameras can see tags.
        if wpilib.RobotBase.isSimulation():
            self.drivetrain.resetOdometry(self.goal.transformBy(Transform2d(-2.0, 1.0, Rotation2d())))

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
