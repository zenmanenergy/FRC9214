#!/usr/bin/env python3
"""
EXAMPLE: Drive up to an AprilTag

Hold A and the robot drives itself to a spot 1 meter in front of an AprilTag,
facing it. The tag's position comes from the field map; the robot's position
comes from the wheels, gyro and cameras together. Good for lining up to score.

Setup first: VISION_ENABLED = True in robot_settings.py (with measured
cameras, measured=True), remove the # before photonlibpy in pyproject.toml,
run py -m robotpy sync, then deploy.

On the robot: put it about 3 m from the tag, a little off to the side, where
the cameras can see tags, with a clear path. Enable in Autonomous: it drives
itself to 1 m in front of the tag, slowly.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                             # command-based robot library
from commands2.button import CommandXboxController                           # an Xbox controller with buttons

import robot_settings                                                        # load robot_settings.py (your robot's settings)
from easyswerve import Autos, Drivetrain                                     # drivetrain, and PathPlanner autos

TAG_ID = None                                                                # which tag; None = the first tag on the field
STOP_DISTANCE_M = 1.0                                                        # how far in front of the tag to stop


class MyRobot(commands2.TimedCommandRobot):                                  # our robot: runs 50 times a second
    def robotInit(self) -> None:                                             # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                       # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                                # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                        # the driver's Xbox controller
        self.vision = self.drivetrain.vision                                 # the cameras (set up from robot_settings.py)
        self.autos = Autos(self.drivetrain)                                  # PathPlanner: plans the path to the tag
        self.drivetrain.driveWithController(self.controller)                 # left stick drives, right stick turns
        self.goal = self.vision.poseInFrontOfTag(TAG_ID, STOP_DISTANCE_M)    # the spot in front of the tag
        if self.goal is None:                                                # None = vision is off
            print("Vision is off: set VISION_ENABLED = True in robot_settings.py")
        else:                                                                # otherwise:
            x, y = self.goal.X(), self.goal.Y()                              # the goal's position on the field (m)
            facing = self.goal.rotation().degrees()                          # and which way to face
            print(f"Goal: x={x:.2f} y={y:.2f} facing {facing:.0f} degrees")
            self.controller.a().whileTrue(self.driveToGoal())                # hold A: drive there by itself
        self.autoCommand = None                                              # no auto yet (None = nothing)

    def driveToGoal(self) -> commands2.Command:                              # a command: drive to the goal
        x, y = self.goal.X(), self.goal.Y()
        facing = self.goal.rotation().degrees()
        return self.autos.driveTo(x, y, facing, maxSpeedMps=1.5)             # slowly (1.5 m/s at most)

    def makeAuto(self):                                                      # builds what autonomous will do
        if self.goal is None:                                                # vision off: nowhere to go
            return None                                                      # None = no auto
        return commands2.SequentialCommandGroup(                             # these run one after another:
            self.driveToGoal(),                                              # 1. drive to the goal
            commands2.PrintCommand("Arrived in front of the tag"),           # 2. say so
        )

    def autonomousInit(self) -> None:                                        # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                                   # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                     # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()             # the part that runs commands
            scheduler.schedule(self.autoCommand)                             # start the auto

    def teleopInit(self) -> None:                                            # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                     # if an auto is still running...
            self.autoCommand.cancel()                                        # stop the auto: the driver takes over

    def testInit(self) -> None:                                              # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()                 # stop every command that is running
