#!/usr/bin/env python3
"""
SETUP STEP 8: Tune the drive motors

The drive motors are tuned differently from the steering. A wheel's speed
mostly depends on voltage in a simple, steady way, so instead of
Ziegler-Nichols the robot measures that relationship (feedforward):
  kS  volts just to get the wheel moving (friction)
  kV  volts for each meter per second of speed
then adds a small P to correct what's left. The robot drives itself forward
and back, as far as the practice carpet allows (up to 2 m), to measure them,
and saves the result.

On the robot: wheels ON the floor. On the practice carpet (its size is in
robot_settings.py), put the robot at one end, facing the long way, centered
side to side, back bumper about 20 cm from the edge. Someone at the Disable button. Driver
Station in Test mode, then Enable. It takes about 15 seconds. Read the
console.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                               # command-based robot library

from easyswerve import Drivetrain                                              # the swerve drivetrain (4 modules)


class MyRobot(commands2.TimedCommandRobot):                                    # our robot: runs 50 times a second
    def robotInit(self) -> None:                                               # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                         # the autotuner is part of the drivetrain
        self.autoCommand = None                                                # no auto yet (None = nothing)
        self.tuneCommand = None                                                # the tuning command (None = not started)

    def testInit(self) -> None:                                                # runs once when enabled in Test: start tuning
        commands2.CommandScheduler.getInstance().cancelAll()                   # stop every command that is running
        print("Drive autotune: forward, then back.")                           # Disable stops it (nothing saved)
        self.tuneCommand = self.drivetrain.autotuner.driveCommand()            # the drive tuning command
        commands2.CommandScheduler.getInstance().schedule(self.tuneCommand)    # start it

    def makeAuto(self):                                                        # builds what autonomous will do
        return None                                                            # this step runs in Test mode

    def autonomousInit(self) -> None:                                          # runs once when enabled in Autonomous
        print("This step runs in Test mode: switch the")
        print("Driver Station to Test and enable.")

    def teleopInit(self) -> None:                                              # runs once when enabled in Teleoperated
        pass                                                                   # pass = do nothing
