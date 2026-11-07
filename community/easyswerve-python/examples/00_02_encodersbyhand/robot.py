#!/usr/bin/env python3
"""
SETUP STEP 2: Turn the wheels by hand

With the robot DISABLED, turn one module's steering by hand, or roll one
wheel by hand. The console says which module the robot thinks you moved, and
which way. That checks three things before any motor runs:
  - each steering encoder is plugged into the DIO port robot_settings.py says
  - each steering encoder counts the right way
  - each drive SPARK MAX belongs to the module robot_settings.py says

On the robot: deploy, keep it DISABLED, and for each module in turn:
  1. Turn its steering slowly COUNTERCLOCKWISE (seen from above), about a
     quarter turn. The console should name THAT module and say
     "counterclockwise".
  2. Roll its wheel a little by hand. The console should name THAT module.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                          # command-based robot library
import wpilib                                                             # WPILib: the library all FRC robots use

from easyswerve import Drivetrain                                         # the swerve drivetrain (4 modules)

STEER_DEG = 5.0                                                           # report a steering move bigger than this (degrees)
ROLL_M = 0.01                                                             # report a wheel roll bigger than this (meters)
STILL_S = 0.3                                                             # a move is over once nothing changes this long (s)


def angleChange(new: float, old: float) -> float:                         # smallest change from old to new, degrees
    return (new - old + 180.0) % 360.0 - 180.0                            # -180 to 180: 359 to 1 is +2, not -358


class MoveWatcher:                                                        # adds up one movement, says when it stopped
    def __init__(self, tiny: float) -> None:                              # runs once, when a MoveWatcher is made
        self.tiny = tiny                                                  # changes smaller than this are just noise
        self.total = 0.0                                                  # how far it has moved in this movement
        self.stillSince = 0.0                                             # when it last moved (seconds)

    def update(self, change: float, now: float):                          # call every loop with the latest change
        if abs(change) > self.tiny:                                       # it moved (more than noise)?
            self.total += change                                          # add it to this movement's total
            self.stillSince = now                                         # and remember when
            return None                                                   # still moving: nothing to report yet
        if self.total != 0.0 and now - self.stillSince >= STILL_S:        # moved, then still for a while?
            total, self.total = self.total, 0.0                           # take the total, start over at 0
            return total                                                  # report the whole movement
        return None                                                       # nothing to report


class MyRobot(commands2.TimedCommandRobot):                               # our robot: runs 50 times a second
    def robotInit(self) -> None:                                          # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                    # no controller here: nothing moves
        names = [name for name, _ in self.drivetrain.modules]             # ["FL", "FR", "RL", "RR"]
        self.steer = {n: MoveWatcher(0.2) for n in names}                 # a watcher for each steering encoder
        self.wheel = {n: MoveWatcher(0.0005) for n in names}              # and for each wheel encoder
        self.lastSteer = {}                                               # each module's last steering angle
        self.lastWheel = {}                                               # each module's last wheel distance
        self.autoCommand = None                                           # no auto yet (None = nothing)

    def robotPeriodic(self) -> None:                                      # runs 50 times a second, in every mode
        now = wpilib.Timer.getFPGATimestamp()                             # the time now, in seconds
        for name, module in self.drivetrain.modules:                      # repeat for each of the 4 modules
            self.checkSteering(name, module, now)                         # did its steering move?
            self.checkWheel(name, module, now)                            # did its wheel roll?

    def checkSteering(self, name, module, now) -> None:                   # report a finished steering move
        angle = module.absoluteDeg()                                      # the Through Bore Encoder, degrees
        if angle is None:                                                 # no signal from the encoder?
            return                                                        # (00_01_checkup reports that)
        last = self.lastSteer.get(name, angle)                            # last angle (this one, the first time)
        change = angleChange(angle, last)                                 # how much it turned since last loop
        self.lastSteer[name] = angle                                      # remember this angle for next time
        moved = self.steer[name].update(change, now)                      # a whole movement, once it's over
        if moved is None or abs(moved) < STEER_DEG:
            return
        way = "counterclockwise" if moved > 0 else "CLOCKWISE"            # + = counterclockwise from above
        enc = module.turningEncoder                                       # this module's steering encoder
        where = "SPARK"                                                   # plugged into the SPARK MAX...
        if enc.location == "rio":                                         # ...or into the roboRIO?
            where = "DIO " + str(enc.dioChannel)                          # then say which DIO port
        msg = f"{name} steering ({where}) turned {moved:+.0f} deg"        # f"..." puts values in the {}
        print(msg, "=", way)                                              # e.g. "... +85 deg = counterclockwise"

    def checkWheel(self, name, module, now) -> None:                      # report a finished wheel roll
        meters = module.wheelMeters()                                     # the drive motor's encoder
        change = meters - self.lastWheel.get(name, meters)                # how far since last loop
        self.lastWheel[name] = meters                                     # remember for next time
        moved = self.wheel[name].update(change, now)                      # a whole roll, once it's over
        if moved is None or abs(moved) < ROLL_M:                          # nothing, or too small?
            return                                                        # then don't print anything
        can = module.drivingSpark.getDeviceId()                           # the drive SPARK MAX's CAN ID
        print(f"{name} wheel (drive CAN {can}) rolled {moved:+.2f} m")    # +.2f: sign, 2 decimals

    def makeAuto(self):                                                   # builds what autonomous will do
        return None                                                       # nothing moves: turn the wheels by hand

    def autonomousInit(self) -> None:                                     # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                                # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                  # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()          # the part that runs commands
            scheduler.schedule(self.autoCommand)                          # start the auto

    def teleopInit(self) -> None:                                         # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                  # if an auto is still running...
            self.autoCommand.cancel()                                     # stop the auto: the driver takes over

    def testInit(self) -> None:                                           # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()              # stop every command that is running
