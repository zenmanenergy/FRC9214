#!/usr/bin/env python3
"""
SETUP STEP 11: Tune driving to a spot (position and heading PID)

driveDistance, turnTo, heading hold, driveTo and the PathPlanner autos all
work the same way: every loop, compare where the robot IS with where it
SHOULD be, and ask for the speed that closes the gap. That is a PID loop
on position (and one on heading). This step finds its P, I and D for you,
two ways, and keeps the one that stops closest to the target:
  1. the model: measure how quickly the robot follows a sudden speed,
     then the SIMC rules give P, I and D
  2. Ziegler-Nichols: raise P until the robot rocks back and forth by the
     same amount every time (like setup step 7, but for the whole robot)
Each set then does real moves, 1 m out and back (less on a short carpet)
and 90 degrees and back,
and the console shows how close each one stopped.

On the robot: run setup steps 7 and 8 first. Wheels ON the floor. On the
practice carpet (its size is in robot_settings.py), put the robot at one
end, facing the long way, centered side to side, back bumper about 20 cm
from the edge (so it can turn on the spot). Someone at the Disable button. Driver Station in Test mode, then Enable. It takes about
a minute and a half. Read the console. Then restart the robot code (or
deploy again) so driveTo and the autos use the new gains too.

Then try it: enable in Autonomous. The robot drives 1 m forward (less on a
short carpet), turns left 90 degrees, turns back and drives back, and prints how far from
each target it stopped.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                               # command-based robot library
from wpimath.geometry import Pose2d                                            # a spot on the field: x, y and heading

from easyswerve import Drivetrain                                              # the swerve drivetrain (4 modules)
from easyswerve import carpet                                                  # the practice carpet's size (robot_settings.py)


class MyRobot(commands2.TimedCommandRobot):                                    # our robot: runs 50 times a second
    def robotInit(self) -> None:                                               # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                         # the autotuner is part of the drivetrain
        self.autoCommand = None                                                # no auto yet (None = nothing)
        self.tuneCommand = None                                                # the tuning command (None = not started)

    def testInit(self) -> None:                                                # runs once when enabled in Test: start tuning
        commands2.CommandScheduler.getInstance().cancelAll()                   # stop every command that is running
        print("Position autotune: about 1 m forward and back,")                # Disable stops it (nothing saved)
        print("then turning on the spot.")
        tuner = self.drivetrain.autotuner                                      # the autotuner (part of the drivetrain)
        self.tuneCommand = tuner.positionCommand()                             # the position tuning command
        commands2.CommandScheduler.getInstance().schedule(self.tuneCommand)    # start it

    def report(self, what, x, heading) -> None:                                # prints how close it stopped
        pose = self.drivetrain.getPose()                                       # where the robot thinks it is now
        cm = abs(pose.X() - x) * 100                                           # meters to centimeters
        deg = abs(self.drivetrain.getHeading() - heading)                      # degrees off
        print(f"{what}: {cm:.1f} cm and {deg:.1f} degrees off")

    def check(self, what, x, heading) -> commands2.Command:                    # a command that runs report()
        return commands2.InstantCommand(lambda: self.report(what, x, heading))

    def makeAuto(self):                                                        # builds what autonomous will do
        self.drivetrain.zeroHeading()                                          # gyro: facing 0 degrees now
        self.drivetrain.resetOdometry(Pose2d())                                # start here: x = 0, y = 0
        meters = carpet.fitMeters(1.0, carpet.END_AHEAD_M)                     # 1 m, or less to fit the carpet
        d = self.drivetrain                                                    # shorter to type
        return commands2.SequentialCommandGroup(                               # these run one after another:
            d.driveDistance(meters),                                           # 1. forward
            self.check("forward", meters, 0.0),
            d.turnTo(90),                                                      # 2. face left
            self.check("turn left", meters, 90.0),
            d.turnTo(0),                                                       # 3. face forward again
            self.check("turn back", meters, 0.0),
            d.driveDistance(-meters),                                          # 4. back to the start
            self.check("back", 0.0, 0.0),
        )

    def autonomousInit(self) -> None:                                          # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                                     # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                       # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()               # the part that runs commands
            scheduler.schedule(self.autoCommand)                               # start the auto

    def teleopInit(self) -> None:                                              # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                       # if an auto is still running...
            self.autoCommand.cancel()                                          # stop the auto: the driver takes over
