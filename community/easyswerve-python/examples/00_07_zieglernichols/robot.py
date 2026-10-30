#!/usr/bin/env python3
"""
SETUP STEP 7: Tune the steering PID with Ziegler-Nichols

The Ziegler-Nichols "ultimate gain" method, done automatically on all four
steering modules:
  1. Measure how the steering responds now (before).
  2. Use only P. Make small steering steps, raising P each time, until the
     wheel stops settling and keeps swinging back and forth by the same
     amount. That P is the ultimate gain Ku; one back-and-forth takes Tu.
  3. Turn Ku and Tu into P, I and D with Ziegler and Nichols' table (RULES
     below), try them, and measure again (after).
  4. If the new gains are good, save them on the roboRIO (loaded at every
     startup from now on).

Do 00_03 first: a backwards motor or encoder makes the wheel run away.

On the robot: wheels ON the floor (that's the load they steer in a match),
robot on a clear patch of floor. Enable in Autonomous. The wheels wiggle
and buzz for about 20 seconds; the robot doesn't drive. Read the console.

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy
"""

import dataclasses

import commands2

from easyswerve import Drivetrain
from easyswerve.tuning.zieglernichols import SteeringStepCheck, ZieglerNicholsTest

# Ziegler and Nichols' table: P = kP x Ku, Ti = ti x Tu, Td = td x Tu
# (None = that part is not used)
RULES = {
    #                  kP     ti     td
    "P only":         (0.50,  None,  None),
    "PI":             (0.45,  0.83,  None),
    "PID (classic)":  (0.60,  0.50,  0.125),
    "some overshoot": (0.33,  0.50,  0.33),
    "no overshoot":   (0.20,  0.50,  0.33),
}
RULE = "no overshoot"  # which row to use; try the others!

I_ZONE_DEG = 3.0  # the I part only works within this many degrees of the target
GOOD_OVERSHOOT_PCT = 25.0  # save the new gains only if overshoot is under this


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()

        # The swings are fast, so the test reads the steering every 5 ms
        # instead of every 20 ms (the normal robot loop)
        self.test = None
        self.addPeriodic(self.sampleSteering, 0.005)

        self.before = {}
        self.after = {}
        self.newGains = None
        self.autoCommand = None

    def sampleSteering(self) -> None:
        if self.test is not None:
            self.test.sample()

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        self.before, self.after, self.newGains = {}, {}, None
        self.test = ZieglerNicholsTest(self.drivetrain, onDone=self.useResults)
        return commands2.SequentialCommandGroup(
            commands2.PrintCommand("Step 1: steering steps with the gains it has now..."),
            SteeringStepCheck(self.drivetrain, onDone=self.before.update),
            commands2.PrintCommand("Step 2: raising P until the wheels keep swinging..."),
            self.test,
            commands2.PrintCommand("Step 4: steering steps with the new gains...").onlyIf(
                lambda: self.newGains is not None
            ),
            SteeringStepCheck(self.drivetrain, onDone=self.after.update).onlyIf(
                lambda: self.newGains is not None
            ),
            commands2.InstantCommand(self.finish),
        )

    def useResults(self, results) -> None:
        """Step 3: the test is done. Ku and Tu -> P, I, D."""
        print("Ultimate gain Ku and period Tu of each module:")
        found = []
        problems = []
        for r in results:
            if r.ku is None:
                print(f"  {r.name}: no result")
                if r.problem not in problems:
                    problems.append(r.problem)
            else:
                print(f"  {r.name}: Ku = {r.ku:6.2f}   Tu = {r.tu * 1000:5.1f} ms   ({len(r.trials)} tries)")
                found.append(r)
        if len(found) < len(results):
            for problem in problems:
                print(f"  Why: {problem}")
            print("Not every module gave a result, so the gains are unchanged.")
            return

        # All four share one set of gains, so use the module that starts
        # swinging first (smallest Ku): safe for all of them.
        limit = min(found, key=lambda r: r.ku)
        ku, tu = limit.ku, limit.tu
        if max(r.ku for r in found) > 1.5 * ku:
            print(f"  Note: {limit.name} swings much sooner than the others. Check it for binding.")

        kP, ti, td = RULES[RULE]
        p = kP * ku
        i = p / (ti * tu) if ti else 0.0  # I = P / Ti
        d = p * (td * tu) if td else 0.0  # D = P x Td
        print(f'Rule "{RULE}": P = {p:.3f}   I = {i:.2f}   D = {d:.5f}')

        self.newGains = dataclasses.replace(
            self.drivetrain.gains,
            steerP=p,
            steerI=i,
            steerD=d,
            steerIZoneDeg=I_ZONE_DEG if i else 0.0,
            source=f"Ziegler-Nichols ({RULE})",
        )
        self.oldGains = self.drivetrain.gains
        self.drivetrain.applyGains(self.newGains)  # try them (not saved yet)

    def finish(self) -> None:
        if self.newGains is None:
            print("No new gains. Read the messages above.")
            return

        def describe(report):
            settle = report["settleS"]
            took = f"{settle:.2f} s" if settle is not None else "never settled"
            return f"overshoot {report['overshootPct']:.0f} %, settled in {took}"

        print(f"Before: {describe(self.before)}")
        print(f"After:  {describe(self.after)}")
        good = self.after["settleS"] is not None and self.after["overshootPct"] < GOOD_OVERSHOOT_PCT
        if good:
            path = self.drivetrain.saveGains(self.newGains)
            print(f"Saved to {path}. These are used from now on.")
        else:
            self.drivetrain.applyGains(self.oldGains)
            print('Not good enough, so NOT saved (back to the old gains). Try RULE = "no overshoot".')

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
