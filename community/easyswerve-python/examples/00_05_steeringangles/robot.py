#!/usr/bin/env python3
"""
SETUP STEP 5: Point all four wheels the same way

Now the steering uses its PID. Every wheel is told to point the same way
(straight ahead, left, right, diagonal...) and the console shows how close
each one got and how long it took. All four should end up parallel, within
a couple of degrees, at about the same time. This is the "do the wheels turn
in sync, to the right angle" test.

On the robot: robot on blocks (wheels off the floor). Enable in Teleoperated
and press the D-pad: up = straight ahead, left = 90 left, right = 90 right,
down = backwards (180). Or enable in Autonomous to go through them all.
LOOK: are all four wheels parallel and pointing the way the console says?

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                             # command-based robot library
import wpilib                                                                # WPILib: the library all FRC robots use
from commands2.button import CommandXboxController                           # an Xbox controller with buttons

import robot_settings                                                        # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                            # the swerve drivetrain (4 modules)

CLOSE_DEG = 2.0                                                              # "there" means within this many degrees


def angleError(target: float, actual: float) -> float:                       # smallest difference between two angles
    return (target - actual + 180.0) % 360.0 - 180.0                         # in degrees, -180 to 180


class MyRobot(commands2.TimedCommandRobot):                                  # our robot: runs 50 times a second
    def robotInit(self) -> None:                                             # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                       # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                                # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                        # the driver's Xbox controller
        self.controller.povUp().onTrue(self.pointAll(0))                     # D-pad up: all wheels straight ahead
        self.controller.povLeft().onTrue(self.pointAll(90))                  # D-pad left: all pointing left
        self.controller.povRight().onTrue(self.pointAll(-90))                # D-pad right: all pointing right
        self.controller.povDown().onTrue(self.pointAll(180))                 # D-pad down: all pointing backwards
        self.autoCommand = None                                              # no auto yet (None = nothing)

    def pointAll(self, degrees, seconds=1.5) -> commands2.Command:           # a command: point all 4 wheels
        return commands2.FunctionalCommand(                                  # a command built from 4 functions:
            lambda: self.start(degrees),                                     # 1. at the start
            lambda: self.watch(degrees),                                     # 2. every loop
            lambda interrupted: self.report(degrees),                        # 3. at the end
            lambda: False,                                                   # 4. finished? (never: the timeout ends it)
            self.drivetrain,                                                 # it uses the drivetrain
        ).withTimeout(seconds)                                               # stop after this many seconds

    def start(self, degrees: float) -> None:                                 # at the start: tell every wheel where to point
        self.startTime = wpilib.Timer.getFPGATimestamp()
        self.lastOff = {}                                                    # module -> last time (s) it was NOT close
        self.distance = {}                                                   # module -> how far it had to turn (degrees)
        for name, module in self.drivetrain.modules:                         # repeat for each of the 4 modules
            now = module.angleRelativeToRobotDeg()                           # where it points now
            self.distance[name] = abs(angleError(degrees, now))              # how far it has to go
            module.pointAt(degrees)                                          # steer there (with PID)

    def watch(self, degrees: float) -> None:                                 # every loop: remember when each wheel was off
        now = wpilib.Timer.getFPGATimestamp()                                # the time now, in seconds
        for name, module in self.drivetrain.modules:                         # repeat for each of the 4 modules
            module.pointAt(degrees)                                          # keep steering there
            actual = module.angleRelativeToRobotDeg()                        # where it points now
            error = angleError(degrees, actual)                              # how far off it is
            if abs(error) > CLOSE_DEG:                                       # not close yet?
                self.lastOff[name] = now - self.startTime                    # remember when

    def report(self, degrees: float) -> None:                                # at the end: how did each wheel do?
        print(f"---- all wheels to {degrees:+.0f} deg ----")
        self.arrived = {}                                                    # module -> when it settled (seconds)
        for name, module in self.drivetrain.modules:                         # repeat for each of the 4 modules
            actual = module.angleRelativeToRobotDeg()                        # where it points now
            if abs(angleError(degrees, actual)) <= CLOSE_DEG:                # close now?
                self.arrived[name] = self.lastOff.get(name, 0.0) + 0.02      # settled after it was last off
        problems = 0                                                         # count the wheels that didn't make it
        for name, module in self.drivetrain.modules:                         # repeat for each of the 4 modules
            actual = module.angleRelativeToRobotDeg()
            error = angleError(degrees, actual)                              # how far off it ended up
            took = self.arrived.get(name)                                    # when it settled (None = never)
            line = f"  {name}: off by {error:+5.1f} deg, "                   # start the line of text...
            if took is not None:                                             # ...and finish it
                line += f"settled in {took:.2f} s"                           # += adds to the end
            else:                                                            # otherwise:
                line += "NEVER got there"
            print(line)
            if took is None or abs(error) > CLOSE_DEG:                       # didn't make it?
                problems += 1                                                # count it

        speeds = {}                                                          # module -> turning speed (degrees per second)
        for n, t in self.arrived.items():                                    # for each wheel that settled
            if t > 0:
                speeds[n] = self.distance[n] / t                             # distance / time = speed
        bigMove = len(speeds) == 4 and min(self.distance.values()) > 30      # fair to compare?
        if problems:                                                         # any wheel off?
            print("  PROBLEM: go back to 00_03_steeronebyone")
            print("  (directions) and 00_04_zerosteering (zeros).")
        elif bigMove and max(speeds.values()) > 2 * min(speeds.values()):    # one 2x slower?
            slow = min(speeds, key=speeds.get)                               # the slowest wheel
            print(f"  All got there, but {slow} steers much slower:")
            print("  it may be binding or have a loose belt.")
        else:                                                                # all good
            print("  All four together. LOOK: parallel, pointing that way?")
            print("  If one points exactly backwards, it was zeroed")
            print("  with its bevel gear on the wrong side.")

    def makeAuto(self):                                                      # builds what autonomous will do
        return commands2.SequentialCommandGroup(                             # every direction in turn:
            self.pointAll(0),                                                # straight ahead
            self.pointAll(90),                                               # left
            self.pointAll(-90),                                              # right
            self.pointAll(45),                                               # diagonal
            self.pointAll(180),                                              # backwards
            self.pointAll(0),                                                # and straight again
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
