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

Try it in the simulator: click Autonomous and read the terminal.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import math

import commands2
import wpilib
from commands2.button import CommandXboxController

import robot_settings
from easyswerve import Drivetrain

CLOSE_DEG = 2.0  # "there" means within this many degrees


def angleError(target: float, actual: float) -> float:
    """Smallest difference between two angles, degrees (-180 to 180)."""
    return (target - actual + 180.0) % 360.0 - 180.0


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # D-pad: point all wheels one way
        self.controller.povUp().onTrue(self.pointAll(0))
        self.controller.povLeft().onTrue(self.pointAll(90))
        self.controller.povRight().onTrue(self.pointAll(-90))
        self.controller.povDown().onTrue(self.pointAll(180))

        self.autoCommand = None

    def pointAll(self, degrees: float, seconds: float = 1.5) -> commands2.Command:
        """Point every wheel at `degrees` (0 = straight ahead, + = left),
        hold for `seconds`, then report."""
        return commands2.FunctionalCommand(
            lambda: self.start(degrees),
            lambda: self.watch(degrees),
            lambda interrupted: self.report(degrees),
            lambda: False,
            self.drivetrain,
        ).withTimeout(seconds)

    def start(self, degrees: float) -> None:
        self.startTime = wpilib.Timer.getFPGATimestamp()
        self.lastOff = {}  # module name -> last time (s) it was NOT close
        self.distance = {}  # module name -> how far it had to turn, degrees
        for name, module in self.drivetrain.modules:
            self.distance[name] = abs(angleError(degrees, module.angleRelativeToRobotDeg()))
            module.pointAt(degrees)

    def watch(self, degrees: float) -> None:
        """Every loop: note the last time each wheel was not close. After that
        it stayed close, so that's when it "got there" (settled)."""
        now = wpilib.Timer.getFPGATimestamp()
        for name, module in self.drivetrain.modules:
            module.pointAt(degrees)
            error = angleError(degrees, module.angleRelativeToRobotDeg())
            if abs(error) > CLOSE_DEG:
                self.lastOff[name] = now - self.startTime

    def report(self, degrees: float) -> None:
        print(f"---- all wheels to {degrees:+.0f} deg ----")
        # settled = just after the last time it was off (if it is close now)
        self.arrived = {}
        for name, module in self.drivetrain.modules:
            if abs(angleError(degrees, module.angleRelativeToRobotDeg())) <= CLOSE_DEG:
                self.arrived[name] = self.lastOff.get(name, 0.0) + 0.02
        problems = 0
        for name, module in self.drivetrain.modules:
            error = angleError(degrees, module.angleRelativeToRobotDeg())
            took = self.arrived.get(name)
            line = f"  {name}: off by {error:+5.1f} deg, "
            line += f"settled in {took:.2f} s" if took is not None else "NEVER got there"
            seen = getattr(module, "simSeenSteerDeg", None)  # simulator only
            if seen is not None:
                real = angleError(seen - math.degrees(module.chassisAngularOffset), 0.0)
                line += f"   (simulator: really pointing {real:+.0f} deg)"
            print(line)
            if took is None or abs(error) > CLOSE_DEG:
                problems += 1

        # Turning speed of each wheel (only fair to compare on a big move)
        speeds = {n: self.distance[n] / t for n, t in self.arrived.items() if t > 0}
        bigMove = len(speeds) == 4 and min(self.distance.values()) > 30
        if problems:
            print("  PROBLEM: go back to 00_03_steeronebyone (directions) and 00_04_zerosteering (zeros).")
        elif bigMove and max(speeds.values()) > 2 * min(speeds.values()):
            slow = min(speeds, key=speeds.get)
            print(f"  All got there, but {slow} steers much slower: it may be binding or have a loose belt.")
        else:
            print("  All four together. LOOK: are they parallel, pointing that way?")
            print("  If one points exactly backwards, it was zeroed with its bevel gear on the wrong side.")

    def makeAuto(self):
        """Builds the command that runs in autonomous: every direction in turn."""
        return commands2.SequentialCommandGroup(
            self.pointAll(0),
            self.pointAll(90),
            self.pointAll(-90),
            self.pointAll(45),
            self.pointAll(180),
            self.pointAll(0),
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
