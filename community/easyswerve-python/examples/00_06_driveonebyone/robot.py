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

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                             # command-based robot library
from commands2.button import CommandXboxController                           # an Xbox controller with buttons

import robot_settings                                                        # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                            # the swerve drivetrain (4 modules)

TEST_VOLTS = 1.5                                                             # slow: about 0.5 m/s
TEST_SECONDS = 1.5                                                           # how long each wheel spins
SETTING_NAMES = {                                                            # short name -> its name in robot_settings.py
    "FL": "FRONT_LEFT",
    "FR": "FRONT_RIGHT",
    "RL": "BACK_LEFT",
    "RR": "BACK_RIGHT",
}
ALL = ["FL", "FR", "RL", "RR"]                                               # all four modules


class MyRobot(commands2.TimedCommandRobot):                                  # our robot: runs 50 times a second
    def robotInit(self) -> None:                                             # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                       # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                                # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                        # the driver's Xbox controller
        self.controller.a().whileTrue(self.testWheels(["FL"]))               # hold A: front left wheel
        self.controller.b().whileTrue(self.testWheels(["FR"]))               # hold B: front right
        self.controller.x().whileTrue(self.testWheels(["RL"]))               # hold X: rear left
        self.controller.y().whileTrue(self.testWheels(["RR"]))               # hold Y: rear right
        self.controller.rightBumper().whileTrue(self.testWheels(ALL))        # right bumper: all four
        self.autoCommand = None                                              # no auto yet (None = nothing)

    def testWheels(self, names) -> commands2.Command:                        # a command: test the named wheels
        straight = commands2.RunCommand(self.straighten, self.drivetrain)    # point all wheels ahead...
        return commands2.SequentialCommandGroup(                             # these run one after another:
            straight.withTimeout(0.5),                                       # 1. ...for half a second
            commands2.FunctionalCommand(                                     # 2. spin them (a command from 4 functions):
                lambda: self.start(names),                                   # at the start
                lambda: self.spin(names),                                    # every loop
                lambda interrupted: self.report(names),                      # at the end
                lambda: False,                                               # finished? (never: the timeout ends it)
                self.drivetrain,                                             # it uses the drivetrain
            ).withTimeout(TEST_SECONDS),                                     # stop after TEST_SECONDS
        )

    def straighten(self) -> None:                                            # every wheel straight ahead, drive motors off
        for _, module in self.drivetrain.modules:                            # repeat for each of the 4 modules
            module.pointAt(0)                                                # 0 = straight ahead (steering PID)

    def start(self, names) -> None:                                          # at the start: remember where each wheel was
        self.startMeters = {}                                                # module -> wheel distance at the start
        self.topSpeed = {}                                                   # module -> fastest speed seen
        for n in names:
            self.startMeters[n] = self.drivetrain.module(n).wheelMeters()
            self.topSpeed[n] = 0.0
        print(f"---- {', '.join(names)}: +{TEST_VOLTS} V. WATCH the wheel. ----")

    def spin(self, names) -> None:                                           # every loop: spin the chosen wheels
        for name in names:
            module = self.drivetrain.module(name)                            # one module by name, e.g. "FL"
            module.spinWheel(TEST_VOLTS)                                     # fixed volts, no PID
            fastest = max(self.topSpeed[name], module.wheelSpeed())          # the bigger of the two
            self.topSpeed[name] = fastest                                    # keep the fastest speed seen

    def report(self, names) -> None:                                         # at the end: how far, how fast?
        for _, module in self.drivetrain.modules:                            # repeat for each of the 4 modules
            module.spinWheel(0.0)                                            # all drive motors off
        for name in names:
            module = self.drivetrain.module(name)                            # one module by name, e.g. "FL"
            rolled = module.wheelMeters() - self.startMeters[name]           # how far it went
            top = self.topSpeed[name]                                        # how fast it got
            print(f"  {name}: encoder says {rolled:+.2f} m, top {top:.2f} m/s")
            if abs(rolled) < 0.05:                                           # less than 5 cm?
                print("    PROBLEM: it hardly moved. Check")
                print(f"    {SETTING_NAMES[name]} drive_can and the motor wires.")

        if len(names) == 1:                                                  # one wheel: check its direction
            setting = SETTING_NAMES[names[0]]
            print("  LOOK at the wheel:")
            print("    rolled forward   -> OK")
            print(f"    rolled backwards -> flip drive_motor_bottom for {setting}")
            print("    a DIFFERENT wheel spun -> drive_can numbers mixed up")
        else:                                                                # all four: did they keep up with each other?
            speeds = [self.topSpeed[n] for n in names]                       # each wheel's top speed
            slowest, fastest = min(speeds), max(speeds)
            if fastest > 0 and (fastest - slowest) / fastest > 0.15:         # over 15 % slower?
                slow = names[speeds.index(slowest)]                          # which one
                print(f"  PROBLEM: {slow} is much slower than the rest.")
                print("  Look for rubbing, a tight bearing or a loose")
                print("  wheel; check its current limit (REV Hardware Client).")
            else:                                                            # otherwise:
                print("  All four about the same speed. LOOK: all forward?")

    def makeAuto(self):                                                      # builds what autonomous will do
        steps = []                                                           # a list of commands to run in order
        for name in ALL:                                                     # each wheel in turn:
            steps.append(self.testWheels([name]))                            # test it
            steps.append(commands2.WaitCommand(0.5))                         # then pause half a second
        steps.append(self.testWheels(ALL))                                   # then all four together
        return commands2.SequentialCommandGroup(*steps)                      # * = use the whole list

    def autonomousInit(self) -> None:                                        # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                                   # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                     # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()             # the part that runs commands
            scheduler.schedule(self.autoCommand)                             # start the auto

    def teleopInit(self) -> None:                                            # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                     # if an auto is still running...
            self.autoCommand.cancel()                                        # stop the auto: the driver takes over
        for _, module in self.drivetrain.modules:                            # repeat for each of the 4 modules
            module.stopMotors()                                              # both of its motors off

    def testInit(self) -> None:                                              # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()                 # stop every command that is running
