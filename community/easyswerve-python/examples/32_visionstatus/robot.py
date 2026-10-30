#!/usr/bin/env python3
"""
EXAMPLE: What do the cameras see?

Two cameras on the Orange Pi (front and back) run PhotonVision, which finds
AprilTags and sends what it sees to the robot over the network. This prints,
once a second, what each camera sees and what the robot does with it.

Setup first: VISION_ENABLED = True in robot_settings.py, and in the simulator
also remove the # before photonlibpy in pyproject.toml, then py -m robotpy sync.

Try it in the simulator: click Autonomous. The robot starts in front of an
AprilTag and spins slowly, so each camera sees tags in turn. Watch the terminal.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import commands2
import wpilib
from commands2.button import CommandXboxController

import robot_settings
from easyswerve import Drivetrain


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Left stick moves the robot, right stick turns it
        self.drivetrain.driveWithController(self.controller)

        # The cameras, set up by the Drivetrain from robot_settings.py
        self.vision = self.drivetrain.vision

        self.lastPrint = 0.0
        self.autoCommand = None

    def robotPeriodic(self) -> None:
        """Runs 50 times a second in every mode."""
        now = wpilib.Timer.getFPGATimestamp()
        if now - self.lastPrint < 1.0:
            return
        self.lastPrint = now

        # The overall answer: where the robot's position comes from right now
        print(f"Vision: {self.vision.status}")

        # Each camera
        for name in self.vision.cameraNames():
            if not self.vision.isConnected(name):
                print(f"  {name:6} not connected")
                continue
            tags = self.vision.tagsSeen(name)
            print(f"  {name:6} sees tags {tags if tags else 'none'}")

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        return commands2.SequentialCommandGroup(
            commands2.InstantCommand(self.startInFrontOfTag),
            self.drivetrain.driveFor(10.0, turn=0.1),  # slow spin: cameras sweep around
        )

    def startInFrontOfTag(self) -> None:
        # In the simulator, put the robot 2 m in front of the first tag on the
        # field. On the real robot, just place it where a camera can see a tag.
        if wpilib.RobotBase.isSimulation():
            spot = self.vision.poseInFrontOfTag(distanceM=2.0)
            if spot is not None:
                self.drivetrain.resetOdometry(spot)

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
