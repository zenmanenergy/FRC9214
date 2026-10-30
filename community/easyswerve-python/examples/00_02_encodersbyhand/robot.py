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

Try it in the simulator: there are no hands, so click Autonomous and the
simulator turns each module's steering, then each wheel, one at a time.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import commands2
import wpilib

from easyswerve import Drivetrain

STEER_DEG = 5.0  # report a steering move bigger than this (degrees)
ROLL_M = 0.01  # report a wheel roll bigger than this (meters)
STILL_S = 0.3  # a move is over once nothing changes for this long (seconds)


def angleChange(new: float, old: float) -> float:
    """Smallest change from old to new, in degrees (-180 to 180)."""
    return (new - old + 180.0) % 360.0 - 180.0


class MoveWatcher:
    """Adds up one movement of a sensor and says when it has stopped."""

    def __init__(self, tiny: float) -> None:
        self.tiny = tiny  # changes smaller than this are just noise
        self.last = None
        self.total = 0.0  # how far it has moved in this movement
        self.stillSince = 0.0

    def update(self, change: float, now: float):
        """Feed it the change since last loop. Returns the total movement
        once the movement is over, otherwise None."""
        if abs(change) > self.tiny:
            self.total += change
            self.stillSince = now
            return None
        if self.total != 0.0 and now - self.stillSince >= STILL_S:
            total, self.total = self.total, 0.0
            return total
        return None


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()

        # One watcher per sensor: steering encoder and wheel encoder
        self.steer = {name: MoveWatcher(0.2) for name, _ in self.drivetrain.modules}
        self.wheel = {name: MoveWatcher(0.0005) for name, _ in self.drivetrain.modules}
        self.lastSteer = {}
        self.lastWheel = {}
        self.autoCommand = None

    def robotPeriodic(self) -> None:
        """Runs 50 times a second in every mode: report anything that moved."""
        now = wpilib.Timer.getFPGATimestamp()
        for name, module in self.drivetrain.modules:
            self.checkSteering(name, module, now)
            self.checkWheel(name, module, now)

    def checkSteering(self, name, module, now) -> None:
        angle = module.absoluteDeg()  # the Through Bore Encoder, degrees
        if angle is None:
            return  # no signal (00_01_checkup reports that)
        change = angleChange(angle, self.lastSteer.get(name, angle))
        self.lastSteer[name] = angle
        moved = self.steer[name].update(change, now)
        if moved is None or abs(moved) < STEER_DEG:
            return
        way = "counterclockwise" if moved > 0 else "CLOCKWISE"
        where = f"DIO {module.turningEncoder.dioChannel}" if module.turningEncoder.location == "rio" else "SPARK"
        print(f"{name} steering encoder ({where}) turned {moved:+.0f} deg = {way}")

    def checkWheel(self, name, module, now) -> None:
        meters = module.wheelMeters()  # the drive motor's encoder
        change = meters - self.lastWheel.get(name, meters)
        self.lastWheel[name] = meters
        moved = self.wheel[name].update(change, now)
        if moved is None or abs(moved) < ROLL_M:
            return
        can = module.drivingSpark.getDeviceId()
        print(f"{name} wheel (drive CAN {can}) rolled {moved:+.2f} m")

    def makeAuto(self):
        """Simulator only: pretend to turn things by hand, one at a time."""
        if not wpilib.RobotBase.isSimulation():
            print("On the real robot, stay disabled and turn the wheels by hand.")
            return None
        steps = []
        for name, module in self.drivetrain.modules:
            steps.append(commands2.PrintCommand(f"(simulator) turning {name} steering counterclockwise..."))
            steps.append(self.runFor(lambda m=module: m.spinSteering(1.0), 0.5))
            steps.append(commands2.WaitCommand(0.5))
        for name, module in self.drivetrain.modules:
            steps.append(commands2.PrintCommand(f"(simulator) rolling {name} wheel..."))
            steps.append(self.runFor(lambda m=module: m.spinWheel(1.0), 0.4))
            steps.append(commands2.WaitCommand(1.0))  # let it coast to a stop
        return commands2.SequentialCommandGroup(*steps)

    def runFor(self, action, seconds: float) -> commands2.Command:
        """Do action every loop for some seconds, then stop all motors."""
        return commands2.FunctionalCommand(
            lambda: None,
            action,
            lambda interrupted: self.stopAll(),
            lambda: False,
            self.drivetrain,
        ).withTimeout(seconds)

    def stopAll(self) -> None:
        for _, module in self.drivetrain.modules:
            module.stopMotors()

    def autonomousInit(self) -> None:
        self.stopAll()
        self.autoCommand = self.makeAuto()
        if self.autoCommand is not None:
            commands2.CommandScheduler.getInstance().schedule(self.autoCommand)

    def teleopInit(self) -> None:
        # Stop the auto when the driver takes over
        if self.autoCommand is not None:
            self.autoCommand.cancel()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()
