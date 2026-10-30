#!/usr/bin/env python3
"""
EXAMPLE: Aim at an AprilTag

Hold A and the robot turns to point its front camera straight at an AprilTag,
while the left stick still drives. This uses what the camera sees directly
(how far left or right the tag is in the picture), so it works even before
the cameras are measured.

Setup first: VISION_ENABLED = True in robot_settings.py, and in the simulator
also remove the # before photonlibpy in pyproject.toml, then py -m robotpy sync.

Try it in the simulator: click Autonomous. The robot starts near a tag,
turned away from it, and turns to aim at it.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import commands2
import wpilib
import wpimath
from commands2.button import CommandXboxController
from wpimath.geometry import Pose2d, Rotation2d

import robot_settings
from easyswerve import Drivetrain

CAMERA = "front"  # which camera aims (its name in robot_settings.py)
AIM_GAIN = 0.01  # turn speed per degree the tag is off center


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)
        self.vision = self.drivetrain.vision
        if CAMERA not in self.vision.cameraNames():
            print(f"No camera named '{CAMERA}': is VISION_ENABLED = True in robot_settings.py,")
            print("and does the camera's name there match its name in PhotonVision?")

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        # Hold A: aim at the tag (left stick still moves the robot)
        self.controller.a().whileTrue(
            commands2.RunCommand(self.driveAndAim, self.drivetrain)
        )

        self.autoCommand = None

    def aimTurnSpeed(self) -> float:
        """How fast to turn to center the tag in the camera's picture."""
        tag = self.vision.target(CAMERA)
        if tag is None:
            return 0.0  # no tag in sight: don't turn

        # Yaw: how many degrees the tag is right (+) or left (-) of center.
        # Tag to the right -> turn right (clockwise, which is negative).
        turn = -tag.getYaw() * AIM_GAIN
        return max(-0.3, min(0.3, turn))  # limit to 30 % turning speed

    def driveAndAim(self) -> None:
        forward = -wpimath.applyDeadband(self.controller.getLeftY(), 0.1)
        left = -wpimath.applyDeadband(self.controller.getLeftX(), 0.1)
        self.drivetrain.drive(forward, left, self.aimTurnSpeed(), True)

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        return commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.startNearTag),
            commands2.RunCommand(
                lambda: self.drivetrain.drive(0, 0, self.aimTurnSpeed(), False),
                self.drivetrain,
            ).withTimeout(4.0),
            commands2.PrintCommand("Aimed: is the tag in the middle of the picture?"),
        )

    def startNearTag(self) -> None:
        # In the simulator, start 2.5 m in front of the first tag, turned 20
        # degrees away from it. On the real robot, place it facing near a tag.
        if wpilib.RobotBase.isSimulation():
            spot = self.vision.poseInFrontOfTag(distanceM=2.5)
            if spot is not None:
                turnedAway = spot.rotation() + Rotation2d.fromDegrees(20)
                self.drivetrain.resetOdometry(Pose2d(spot.translation(), turnedAway))

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
