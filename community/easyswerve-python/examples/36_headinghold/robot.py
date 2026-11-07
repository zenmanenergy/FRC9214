#!/usr/bin/env python3
"""
EXAMPLE: Heading hold (keep facing the same way)

No two wheels are exactly the same: one has more friction, one a tighter
bearing. Driving "straight", those differences slowly turn the robot. Heading
hold fixes it: whenever the robot drives without being told to turn, the
navX gyro keeps it facing the way it faced when the turning stopped. It is
on by default (HEADING_HOLD in robot_settings.py).

This auto drives forward 2 seconds (less on a short carpet) with heading
hold OFF, then back with it ON, and prints how much the robot turned each
time. In teleop, press A to switch heading hold off and on, and feel the
difference.

On the robot: put it in the middle of the practice carpet, facing the long
way, and enable in Autonomous. To see a big difference, put a strip of
carpet or tape under one side, or have someone gently push a front corner
while it drives.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                       # command-based robot library
from commands2.button import CommandXboxController                     # an Xbox controller with buttons

import robot_settings                                                  # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                      # the swerve drivetrain (4 modules)
from easyswerve import carpet                                          # the practice carpet's size (robot_settings.py)


class MyRobot(commands2.TimedCommandRobot):                            # our robot: runs 50 times a second
    def robotInit(self) -> None:                                       # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                 # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                          # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                  # the driver's Xbox controller
        self.drivetrain.driveWithController(self.controller)           # left stick drives, right stick turns
        toggle = commands2.InstantCommand(self.toggleHold)             # a command: switch heading hold
        self.controller.a().onTrue(toggle)                             # press A to run it
        self.autoCommand = None                                        # no auto yet (None = nothing)

    def toggleHold(self) -> None:                                      # heading hold on -> off, off -> on
        dt = self.drivetrain                                           # a shorter name for the drivetrain
        dt.headingHold = not dt.headingHold                            # not True = False, not False = True
        print("Heading hold", "ON" if dt.headingHold else "OFF")       # say which

    def startLeg(self, hold: bool) -> None:                            # before each drive: set hold, note heading
        self.drivetrain.headingHold = hold                             # True = on, False = off
        self.holdOn = hold                                             # remember it for the report
        self.startHeading = self.drivetrain.getHeading()               # which way it faces now

    def endLeg(self) -> None:                                          # after each drive: how much did it turn?
        now = self.drivetrain.getHeading()                             # which way it faces now
        turned = (now - self.startHeading + 180.0) % 360.0 - 180.0     # the change, -180 to 180
        label = "ON " if self.holdOn else "OFF"
        print(f"Heading hold {label}: turned {turned:+.1f} deg driving straight")

    def makeAuto(self):                                                # builds what autonomous will do
        dt = self.drivetrain                                           # a shorter name for the drivetrain
        seconds = carpet.fitSeconds(2.0, forward=0.25)                 # 2 s, or less to stay on the carpet
        return commands2.SequentialCommandGroup(                       # these run one after another:
            commands2.InstantCommand(lambda: self.startLeg(False)),    # 1. hold OFF
            dt.driveFor(seconds, forward=0.25),                        # 2. drive forward 2 s
            commands2.InstantCommand(self.endLeg),                     # 3. how much did it turn?
            commands2.WaitCommand(1.0),                                # 4. pause
            commands2.InstantCommand(lambda: self.startLeg(True)),     # 5. hold ON
            dt.driveFor(seconds, forward=-0.25),                       # 6. drive back 2 s
            commands2.InstantCommand(self.endLeg),                     # 7. how much this time?
            commands2.InstantCommand(self.restoreHold),                # 8. back to the setting
        )

    def restoreHold(self) -> None:                                     # heading hold back to robot_settings.py's value
        self.drivetrain.headingHold = robot_settings.HEADING_HOLD

    def autonomousInit(self) -> None:                                  # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                             # build the auto (makeAuto, above)
        if self.autoCommand is not None:                               # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()       # the part that runs commands
            scheduler.schedule(self.autoCommand)                       # start the auto

    def teleopInit(self) -> None:                                      # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                               # if an auto is still running...
            self.autoCommand.cancel()                                  # stop the auto: the driver takes over

    def testInit(self) -> None:                                        # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()           # stop every command that is running
