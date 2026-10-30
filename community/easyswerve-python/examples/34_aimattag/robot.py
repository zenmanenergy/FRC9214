#!/usr/bin/env python3
"""
EXAMPLE: Aim at an AprilTag

Hold A and the robot turns to point its front camera straight at an AprilTag,
while the left stick still drives. This uses what the camera sees directly
(how far left or right the tag is in the picture), so it works even before
the cameras are measured.

Setup first: VISION_ENABLED = True in robot_settings.py, remove the # before
photonlibpy in pyproject.toml, run py -m robotpy sync, then deploy.

On the robot: put it about 2.5 m from an AprilTag, turned a little away from
it (the tag still in the front camera's view). Enable in Autonomous: it turns
in place to aim at the tag.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy
"""

import commands2
import wpimath
from commands2.button import CommandXboxController

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
            commands2.RunCommand(
                lambda: self.drivetrain.drive(0, 0, self.aimTurnSpeed(), False),
                self.drivetrain,
            ).withTimeout(4.0),
            commands2.PrintCommand("Aimed: is the tag in the middle of the picture?"),
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
