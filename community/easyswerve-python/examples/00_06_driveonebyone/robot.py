#!/usr/bin/env python3
"""
SETUP STEP 6: Run each drive motor by itself

Points every wheel straight ahead, then spins ONE wheel at a low, fixed
voltage (no PID) and reports how far and how fast its encoder says it went.
Positive voltage should roll the wheel FORWARD. Then all four spin together:
at the same voltage they should all reach about the same speed. This is the
"do the wheels run in sync" test.

On the robot: robot on blocks (wheels off the floor). Enable in Teleoperated
and HOLD: A = front left, B = front right, X = rear left, Y = rear right,
right bumper = all four. Or enable in Autonomous to test them all in turn.
LOOK: does the wheel roll forward (the top of the wheel moving toward the
front of the robot)?

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy
"""

import commands2
import wpilib
from commands2.button import CommandXboxController

import robot_settings
from easyswerve import Drivetrain

TEST_VOLTS = 1.5  # slow: about 0.5 m/s
TEST_SECONDS = 1.5
SETTING_NAMES = {"FL": "FRONT_LEFT", "FR": "FRONT_RIGHT", "RL": "BACK_LEFT", "RR": "BACK_RIGHT"}


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)

        # Hold a button to test one wheel, or all four
        self.controller.a().whileTrue(self.testWheels(["FL"]))
        self.controller.b().whileTrue(self.testWheels(["FR"]))
        self.controller.x().whileTrue(self.testWheels(["RL"]))
        self.controller.y().whileTrue(self.testWheels(["RR"]))
        self.controller.rightBumper().whileTrue(self.testWheels(["FL", "FR", "RL", "RR"]))

        self.autoCommand = None

    def testWheels(self, names) -> commands2.Command:
        """Point all wheels straight, spin the named ones, then report."""
        return commands2.SequentialCommandGroup(
            # 1. straighten every wheel (steering PID), drive motors off
            commands2.RunCommand(self.straighten, self.drivetrain).withTimeout(0.5),
            # 2. spin the chosen wheels at TEST_VOLTS
            commands2.FunctionalCommand(
                lambda: self.start(names),
                lambda: self.spin(names),
                lambda interrupted: self.report(names),
                lambda: False,
                self.drivetrain,
            ).withTimeout(TEST_SECONDS),
        )

    def straighten(self) -> None:
        for _, module in self.drivetrain.modules:
            module.pointAt(0)

    def start(self, names) -> None:
        self.startMeters = {n: self.drivetrain.module(n).wheelMeters() for n in names}
        self.topSpeed = {n: 0.0 for n in names}
        print(f"---- {', '.join(names)}: drive at +{TEST_VOLTS} V. WATCH the wheel. ----")

    def spin(self, names) -> None:
        for name in names:
            module = self.drivetrain.module(name)
            module.spinWheel(TEST_VOLTS)  # steering stays where straighten() put it
            self.topSpeed[name] = max(self.topSpeed[name], module.wheelSpeed())

    def report(self, names) -> None:
        for _, module in self.drivetrain.modules:
            module.spinWheel(0.0)
        for name in names:
            module = self.drivetrain.module(name)
            rolled = module.wheelMeters() - self.startMeters[name]
            line = f"  {name}: encoder says {rolled:+.2f} m, top speed {self.topSpeed[name]:.2f} m/s"
            print(line)
            if abs(rolled) < 0.05:
                print(f"    PROBLEM: it hardly moved. Check {SETTING_NAMES[name]} drive_can and the motor wires.")

        if len(names) == 1:
            setting = SETTING_NAMES[names[0]]
            print("  LOOK at the wheel:")
            print("    rolled forward   -> OK")
            print(f"    rolled backwards -> flip drive_motor_bottom for {setting}")
            print("    a DIFFERENT wheel spun -> the drive_can numbers are mixed up")
        else:
            speeds = [self.topSpeed[n] for n in names]
            slowest, fastest = min(speeds), max(speeds)
            if fastest > 0 and (fastest - slowest) / fastest > 0.15:
                slow = names[speeds.index(slowest)]
                print(f"  PROBLEM: {slow} is much slower than the rest. Look for rubbing, a tight")
                print("  bearing or a loose wheel; check its SPARK's current limit in REV Hardware Client.")
            else:
                print("  All four reached about the same speed. LOOK: all rolling forward?")

    def makeAuto(self):
        """Builds the command that runs in autonomous: each wheel, then all four."""
        steps = []
        for name in ("FL", "FR", "RL", "RR"):
            steps.append(self.testWheels([name]))
            steps.append(commands2.WaitCommand(0.5))
        steps.append(self.testWheels(["FL", "FR", "RL", "RR"]))
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
