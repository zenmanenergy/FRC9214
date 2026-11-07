#!/usr/bin/env python3
"""
EXAMPLE: What do the cameras see?

Two cameras on the Orange Pi (front and back) run PhotonVision, which finds
AprilTags and sends what it sees to the robot over the network. This prints
what each camera sees and what the robot does with it, whenever that changes.

Setup first: VISION_ENABLED = True in robot_settings.py, remove the # before
photonlibpy in pyproject.toml, run py -m robotpy sync, then deploy.

On the robot: put it about 2 m in front of an AprilTag, with room to spin.
Enable in Autonomous: it spins slowly in place, so the front camera, then
the back camera, sees the tag. Watch the console.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                    # command-based robot library
import wpilib                                                       # WPILib: the library all FRC robots use
from commands2.button import CommandXboxController                  # an Xbox controller with buttons

import robot_settings                                               # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                   # the swerve drivetrain (4 modules)


class MyRobot(commands2.TimedCommandRobot):                         # our robot: runs 50 times a second
    def robotInit(self) -> None:                                    # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                              # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                       # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)               # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)        # left stick drives, right stick turns
        self.vision = self.drivetrain.vision                        # the cameras (set up from robot_settings.py)
        self.lastPrint = 0.0                                        # when we last checked
        self.lastSeen = {}                                          # what we last printed, so we only print changes
        self.autoCommand = None                                     # no auto yet (None = nothing)

    def robotPeriodic(self) -> None:                                # runs 50 times a second, in every mode
        now = wpilib.Timer.getFPGATimestamp()                       # the time now, in seconds
        if now - self.lastPrint < 0.5:                              # checked less than half a second ago?
            return                                                  # then wait
        self.lastPrint = now
        status = self.vision.status                                 # where the position comes from right now
        if status != self.lastSeen.get("status"):                   # changed since last time?
            self.lastSeen["status"] = status                        # remember it
            print(f"Vision: {status}")                              # and print it
        for name in self.vision.cameraNames():                      # repeat for each camera ("front", "back")
            if not self.vision.isConnected(name):                   # camera not sending pictures?
                now_sees = "not connected"
            else:                                                   # otherwise:
                tags = self.vision.tagsSeen(name)                   # the tag numbers it sees now
                now_sees = f"sees tags {tags}" if tags else "sees no tags"
            if now_sees != self.lastSeen.get(name):                 # different from last time?
                self.lastSeen[name] = now_sees                      # remember it
                print(f"  {name:6} {now_sees}")                     # :6 = pad the name to 6 letters

    def makeAuto(self):                                             # builds what autonomous will do
        return self.drivetrain.driveFor(10.0, turn=0.1)             # slow spin: the cameras sweep around

    def autonomousInit(self) -> None:                               # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                          # build the auto (makeAuto, above)
        if self.autoCommand is not None:                            # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()    # the part that runs commands
            scheduler.schedule(self.autoCommand)                    # start the auto

    def teleopInit(self) -> None:                                   # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                            # if an auto is still running...
            self.autoCommand.cancel()                               # stop the auto: the driver takes over

    def testInit(self) -> None:                                     # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()        # stop every command that is running
