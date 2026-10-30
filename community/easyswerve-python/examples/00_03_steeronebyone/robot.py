#!/usr/bin/env python3
"""
SETUP STEP 3: Run each steering motor by itself

Turns ONE module's steering motor at a low, fixed voltage (no PID), then says
what both steering sensors saw:
  - the motor's own sensor (inside the NEO)
  - the Through Bore Encoder (the absolute encoder)
Positive voltage should turn the wheel COUNTERCLOCKWISE seen from above, and
both sensors should count up. When they don't, it tells you which setting in
robot_settings.py to change. This is the step that fixes wheels that fight
each other or point the wrong way.

On the robot: put the robot on blocks (wheels off the floor), enable in
Teleoperated, and HOLD a button: A = front left, B = front right,
X = rear left, Y = rear right. WATCH the wheel: which one turned, and which way?
Or enable in Autonomous to test all four, one after another.

Try it in the simulator: click Autonomous. The simulator also says which way
the wheel really turned (on the robot, you look).

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import commands2
import wpilib
from commands2.button import CommandXboxController

import robot_settings
from easyswerve import Drivetrain

TEST_VOLTS = 1.0  # low: the wheel turns slowly
TEST_SECONDS = 1.0
SETTING_NAMES = {"FL": "FRONT_LEFT", "FR": "FRONT_RIGHT", "RL": "BACK_LEFT", "RR": "BACK_RIGHT"}


class AngleTracker:
    """Adds up how far an angle reading has turned in total (degrees), even
    when it wraps around from 359 to 0."""

    def __init__(self) -> None:
        self.last = None
        self.total = 0.0

    def update(self, degrees) -> None:
        if degrees is None:
            return
        if self.last is not None:
            self.total += (degrees - self.last + 180.0) % 360.0 - 180.0
        self.last = degrees


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Two trackers per module: motor sensor and absolute encoder
        self.motorSensor = {name: AngleTracker() for name, _ in self.drivetrain.modules}
        self.encoder = {name: AngleTracker() for name, _ in self.drivetrain.modules}

        # Hold a button to test one module
        self.controller.a().whileTrue(self.testSteering("FL"))
        self.controller.b().whileTrue(self.testSteering("FR"))
        self.controller.x().whileTrue(self.testSteering("RL"))
        self.controller.y().whileTrue(self.testSteering("RR"))

        self.autoCommand = None

    def robotPeriodic(self) -> None:
        """Runs 50 times a second: keep the trackers up to date."""
        for name, module in self.drivetrain.modules:
            self.motorSensor[name].update(module.steeringDeg())
            self.encoder[name].update(module.absoluteDeg())

    def testSteering(self, name: str) -> commands2.Command:
        """Turn one module's steering at TEST_VOLTS, then report."""
        module = self.drivetrain.module(name)
        return commands2.FunctionalCommand(
            lambda: self.startTest(name),
            lambda: module.spinSteering(TEST_VOLTS),  # every loop
            lambda interrupted: self.endTest(name),
            lambda: False,
            self.drivetrain,
        ).withTimeout(TEST_SECONDS)

    def startTest(self, name: str) -> None:
        # Everything else off, so nothing else moves
        for _, module in self.drivetrain.modules:
            module.stopMotors()
        # Remember where every sensor started
        self.motorStart = {n: t.total for n, t in self.motorSensor.items()}
        self.encoderStart = {n: t.total for n, t in self.encoder.items()}
        self.seenStart = getattr(self.drivetrain.module(name), "simSeenSteerDeg", None)
        print(f"---- {name}: steering motor at +{TEST_VOLTS} V. WATCH the wheel. ----")

    def endTest(self, name: str) -> None:
        module = self.drivetrain.module(name)
        module.stopMotors()
        motor = self.motorSensor[name].total - self.motorStart[name]
        enc = {n: t.total - self.encoderStart[n] for n, t in self.encoder.items()}
        mine = enc[name]
        setting = SETTING_NAMES[name]
        print(f"  motor sensor turned {motor:+.0f} deg, encoder turned {mine:+.0f} deg")

        # Simulator only: what you would have seen
        seen = getattr(module, "simSeenSteerDeg", None)
        if seen is not None and self.seenStart is not None:
            way = "counterclockwise" if seen > self.seenStart else "CLOCKWISE"
            print(f"  (simulator: the {name} wheel really turned {way})")

        # 1. Did the motor turn at all?
        if abs(motor) < 10:
            print(f"  PROBLEM: the motor hardly turned. Check {setting} turn_can, the motor wires,")
            print("  and that nothing is jammed.")
            return

        # 2. Did THIS module's encoder see it?
        if abs(mine) < 0.3 * abs(motor):
            others = [n for n, d in enc.items() if n != name and abs(d) > 0.3 * abs(motor)]
            if others:
                other = self.drivetrain.module(others[0])
                port = other.turningEncoder.dioChannel
                print(f"  PROBLEM: {name}'s encoder didn't move, but {others[0]}'s did (DIO {port}).")
                print(f"  If the {name} wheel turned, its encoder is plugged into DIO {port}:")
                print(f"  set encoder_dio={port} for {setting} (and fix {others[0]} to match).")
                print("  If a DIFFERENT wheel turned, the turn_can numbers are mixed up instead.")
            else:
                print(f"  PROBLEM: {name}'s encoder didn't move. Check its 3 wires and DIO number,")
                print("  and that it is pressed onto the steering shaft.")
            return

        # 3. Do the motor sensor and the encoder agree on the direction?
        if (motor > 0) == (mine > 0):
            print("  Motor and encoder agree. LOOK at the wheel:")
            print("    turned counterclockwise -> OK, this module is right")
            print(f"    turned clockwise        -> both are backwards: flip turn_motor_bottom for {setting}")
            print(f"                               AND flip encoder_reversed for {setting}")
        else:
            print("  PROBLEM: motor and encoder DISAGREE (this makes the wheel fight itself). LOOK:")
            print(f"    turned counterclockwise -> the encoder is backwards: flip encoder_reversed for {setting}")
            print(f"    turned clockwise        -> the motor is backwards: flip turn_motor_bottom for {setting}")
        print("  ('flip' = True becomes False, False becomes True. Deploy and test again after a change.)")

    def makeAuto(self):
        """Builds the command that runs in autonomous: all four in turn."""
        steps = []
        for name in ("FL", "FR", "RL", "RR"):
            steps.append(self.testSteering(name))
            steps.append(commands2.WaitCommand(0.5))
        return commands2.SequentialCommandGroup(*steps)

    def autonomousInit(self) -> None:
        self.autoCommand = self.makeAuto()
        if self.autoCommand is not None:
            commands2.CommandScheduler.getInstance().schedule(self.autoCommand)

    def teleopInit(self) -> None:
        # Stop the auto when the driver takes over
        if self.autoCommand is not None:
            self.autoCommand.cancel()
        for _, module in self.drivetrain.modules:
            module.stopMotors()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()
