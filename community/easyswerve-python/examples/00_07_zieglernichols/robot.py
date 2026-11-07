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

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import dataclasses

import commands2                                                              # command-based robot library

from easyswerve import Drivetrain                                             # the swerve drivetrain (4 modules)
from easyswerve.tuning.zieglernichols import SteeringStepCheck, ZieglerNicholsTest

RULES = {                                                                     # Ziegler and Nichols' table. Columns:
    "P only":         (0.50,  None,  None),                                   # kP: P = kP x Ku
    "PI":             (0.45,  0.83,  None),                                   # ti: Ti = ti x Tu
    "PID (classic)":  (0.60,  0.50,  0.125),                                  # td: Td = td x Tu
    "some overshoot": (0.33,  0.50,  0.33),                                   # None = that part is not used
    "no overshoot":   (0.20,  0.50,  0.33),
}
RULE = "no overshoot"                                                         # which row to use; try the others!

I_ZONE_DEG = 3.0                                                              # I only works this close to the target (degrees)
GOOD_OVERSHOOT_PCT = 25.0                                                     # save new gains only if overshoot is under this


class MyRobot(commands2.TimedCommandRobot):                                   # our robot: runs 50 times a second
    def robotInit(self) -> None:                                              # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                        # no controller: the test moves the wheels itself
        self.test = None                                                      # the tuning test (made in makeAuto)
        self.addPeriodic(self.sampleSteering, 0.005)                          # also run sampleSteering every 5 ms
        self.before = {}                                                      # step results before tuning
        self.after = {}                                                       # and after
        self.newGains = None                                                  # the new gains (None = none yet)
        self.autoCommand = None                                               # no auto yet (None = nothing)

    def sampleSteering(self) -> None:                                         # every 5 ms: the swings are too fast for 20 ms
        if self.test is not None:                                             # if the test exists...
            self.test.sample()                                                # ...let it read the steering angles

    def makeAuto(self):                                                       # builds what autonomous will do
        self.before, self.after, self.newGains = {}, {}, None                 # start fresh each time
        dt = self.drivetrain                                                  # a shorter name for the drivetrain
        self.test = ZieglerNicholsTest(dt, onDone=self.useResults)            # calls useResults when done
        before = SteeringStepCheck(dt, onDone=self.before.update)             # measure steering: before
        after = SteeringStepCheck(dt, onDone=self.after.update)               # and after
        haveGains = lambda: self.newGains is not None                         # True once there are new gains
        return commands2.SequentialCommandGroup(                              # these run one after another:
            commands2.PrintCommand("Step 1: steps with the gains now..."),
            before,                                                           # step 1
            commands2.PrintCommand("Step 2: raising P until it swings..."),
            self.test,                                                        # step 2: Ku and Tu (then step 3: useResults)
            commands2.PrintCommand("Step 4: steps with the new gains..."),
            after.onlyIf(haveGains),                                          # step 4 (only if there are new gains)
            commands2.InstantCommand(self.finish),                            # step 5: save or not
        )

    def useResults(self, results) -> None:                                    # step 3: Ku and Tu -> P, I, D
        print("Ultimate gain Ku and period Tu of each module:")
        found = []                                                            # modules that gave a result
        problems = []                                                         # why the others didn't
        for r in results:                                                     # repeat for each module's result
            if r.ku is None:                                                  # no Ku found?
                print(f"  {r.name}: no result")
                if r.problem not in problems:                                 # each reason only once
                    problems.append(r.problem)
            else:                                                             # otherwise:
                ms = r.tu * 1000                                              # Tu in milliseconds
                print(f"  {r.name}: Ku = {r.ku:6.2f}   Tu = {ms:5.1f} ms")    # :6.2f = 2 decimals
                found.append(r)
        if len(found) < len(results):                                         # a module had no result?
            for problem in problems:
                print(f"  Why: {problem}")
            print("Not every module gave a result: gains unchanged.")
            return                                                            # stop here

        limit = min(found, key=lambda r: r.ku)                                # the module that swings first (smallest Ku)
        ku, tu = limit.ku, limit.tu                                           # all 4 share gains, so use its Ku and Tu
        if max(r.ku for r in found) > 1.5 * ku:                               # much lower than the others?
            print(f"  Note: {limit.name} swings much sooner than the others.")
            print("  Check it for binding.")

        kP, ti, td = RULES[RULE]                                              # the chosen row of the table
        p = kP * ku                                                           # P = kP x Ku
        i = p / (ti * tu) if ti else 0.0                                      # I = P / Ti   (0 if not used)
        d = p * (td * tu) if td else 0.0                                      # D = P x Td   (0 if not used)
        print(f'Rule "{RULE}": P = {p:.3f}   I = {i:.2f}   D = {d:.5f}')

        self.newGains = dataclasses.replace(                                  # a copy of the gains, with these changed:
            self.drivetrain.gains,
            steerP=p,
            steerI=i,
            steerD=d,
            steerIZoneDeg=I_ZONE_DEG if i else 0.0,                           # I zone only if I is used
            source=f"Ziegler-Nichols ({RULE})",                               # a note: where they came from
        )
        self.oldGains = self.drivetrain.gains                                 # keep the old ones, in case
        self.drivetrain.applyGains(self.newGains)                             # try the new ones (not saved yet)

    def finish(self) -> None:                                                 # step 5: compare before and after, save if good
        if self.newGains is None:                                             # no new gains?
            print("No new gains. Read the messages above.")
            return

        def describe(report):                                                 # a step result as one line of text
            settle = report["settleS"]                                        # seconds to settle (None = never)
            took = f"{settle:.2f} s" if settle is not None else "never settled"
            return f"overshoot {report['overshootPct']:.0f} %, settled in {took}"

        print(f"Before: {describe(self.before)}")
        print(f"After:  {describe(self.after)}")
        settled = self.after["settleS"] is not None                           # did it settle?
        calm = self.after["overshootPct"] < GOOD_OVERSHOOT_PCT                # and overshoot little?
        if settled and calm:                                                  # good: keep them
            path = self.drivetrain.saveGains(self.newGains)                   # save on the roboRIO
            print(f"Saved to {path}. These are used from now on.")
        else:                                                                 # not good: undo
            self.drivetrain.applyGains(self.oldGains)                         # back to the old gains
            print("Not good enough, so NOT saved (old gains back).")
            print('Try RULE = "no overshoot".')

    def autonomousInit(self) -> None:                                         # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                                    # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                      # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()              # the part that runs commands
            scheduler.schedule(self.autoCommand)                              # start the auto

    def teleopInit(self) -> None:                                             # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                      # if an auto is still running...
            self.autoCommand.cancel()                                         # stop the auto: the driver takes over

    def testInit(self) -> None:                                               # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()                  # stop every command that is running
