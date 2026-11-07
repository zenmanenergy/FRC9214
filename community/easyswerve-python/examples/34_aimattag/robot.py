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

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                         # command-based robot library
import wpimath                                                           # WPILib's math tools
from commands2.button import CommandXboxController                       # an Xbox controller with buttons

import robot_settings                                                    # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                        # the swerve drivetrain (4 modules)

CAMERA = "front"                                                         # which camera aims (its name in robot_settings.py)
AIM_GAIN = 0.01                                                          # turn speed per degree the tag is off center


class MyRobot(commands2.TimedCommandRobot):                              # our robot: runs 50 times a second
    def robotInit(self) -> None:                                         # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                   # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                            # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                    # the driver's Xbox controller
        self.vision = self.drivetrain.vision                             # the cameras (set up from robot_settings.py)
        if CAMERA not in self.vision.cameraNames():                      # no camera with that name?
            print(f"No camera named {CAMERA}: VISION_ENABLED = True")
            print("in robot_settings.py? Does it match PhotonVision?")
        self.drivetrain.driveWithController(self.controller)             # left stick drives, right stick turns
        aim = commands2.RunCommand(self.driveAndAim, self.drivetrain)    # a command: driveAndAim every loop
        self.controller.a().whileTrue(aim)                               # hold A: aim (left stick still drives)
        self.autoCommand = None                                          # no auto yet (None = nothing)

    def aimTurnSpeed(self) -> float:                                     # how fast to turn to center the tag
        tag = self.vision.target(CAMERA)                                 # what the camera sees now (None = no tag)
        if tag is None:
            return 0.0                                                   # no tag in sight: don't turn
        turn = -tag.getYaw() * AIM_GAIN                                  # yaw + = tag right of center, so turn right (-)
        return max(-0.3, min(0.3, turn))                                 # limit to 30 % turning speed

    def driveAndAim(self) -> None:                                       # every loop while A is held
        pad = self.controller                                            # a shorter name for the controller
        forward = -wpimath.applyDeadband(pad.getLeftY(), 0.1)            # left stick up = forward
        left = -wpimath.applyDeadband(pad.getLeftX(), 0.1)               # left stick left = move left
        turn = self.aimTurnSpeed()                                       # the camera does the turning
        self.drivetrain.drive(forward, left, turn, True)                 # True = field-relative

    def aimInPlace(self) -> None:                                        # turn toward the tag, without driving
        self.drivetrain.drive(0, 0, self.aimTurnSpeed(), False)          # forward 0, left 0, turn

    def makeAuto(self):                                                  # builds what autonomous will do
        aim = commands2.RunCommand(self.aimInPlace, self.drivetrain)     # aimInPlace every loop
        return commands2.SequentialCommandGroup(                         # these run one after another:
            aim.withTimeout(4.0),                                        # 1. aim for 4 seconds
            commands2.PrintCommand("Aimed: is the tag centered?"),       # 2. check the picture
        )

    def autonomousInit(self) -> None:                                    # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                               # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                 # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()         # the part that runs commands
            scheduler.schedule(self.autoCommand)                         # start the auto

    def teleopInit(self) -> None:                                        # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                 # if an auto is still running...
            self.autoCommand.cancel()                                    # stop the auto: the driver takes over

    def testInit(self) -> None:                                          # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()             # stop every command that is running
