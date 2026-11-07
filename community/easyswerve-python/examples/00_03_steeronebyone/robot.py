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

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then deploy: py -m robotpy deploy

The comments on the right explain each line. Python ignores everything
after a # sign, so they don't change what the code does.
"""

import commands2                                                          # command-based robot library
from commands2.button import CommandXboxController                        # an Xbox controller with buttons

import robot_settings                                                     # load robot_settings.py (your robot's settings)
from easyswerve import Drivetrain                                         # the swerve drivetrain (4 modules)

TEST_VOLTS = 1.0                                                          # low: the wheel turns slowly
TEST_SECONDS = 1.0                                                        # how long each test runs
SETTING_NAMES = {                                                         # short name -> its name in robot_settings.py
    "FL": "FRONT_LEFT",
    "FR": "FRONT_RIGHT",
    "RL": "BACK_LEFT",
    "RR": "BACK_RIGHT",
}


class AngleTracker:                                                       # adds up how far an angle has turned in total
    def __init__(self) -> None:                                           # runs once, when a tracker is made
        self.last = None                                                  # the last reading (None = none yet)
        self.total = 0.0                                                  # total turn so far, degrees

    def update(self, degrees) -> None:                                    # call every loop with the newest reading
        if degrees is None:                                               # no reading (encoder not sending)?
            return                                                        # skip it
        if self.last is not None:                                         # if there was a reading before...
            step = (degrees - self.last + 180.0) % 360.0 - 180.0          # ...the change, even across 359 -> 0
            self.total += step                                            # add it to the total
        self.last = degrees                                               # remember this reading


class MyRobot(commands2.TimedCommandRobot):                               # our robot: runs 50 times a second
    def robotInit(self) -> None:                                          # runs ONCE, when the robot code starts
        self.drivetrain = Drivetrain()                                    # make the drivetrain (uses robot_settings.py)
        port = robot_settings.CONTROLLER_PORT                             # Driver Station USB slot (usually 0)
        self.controller = CommandXboxController(port)                     # the driver's Xbox controller
        names = [name for name, _ in self.drivetrain.modules]             # ["FL", "FR", "RL", "RR"]
        self.motorSensor = {n: AngleTracker() for n in names}             # the NEO's own sensor, per module
        self.encoder = {n: AngleTracker() for n in names}                 # the Through Bore Encoder, per module

        self.controller.a().whileTrue(self.testSteering("FL"))            # hold A: test front left
        self.controller.b().whileTrue(self.testSteering("FR"))            # hold B: front right
        self.controller.x().whileTrue(self.testSteering("RL"))            # hold X: rear left
        self.controller.y().whileTrue(self.testSteering("RR"))            # hold Y: rear right

        self.autoCommand = None                                           # no auto yet (None = nothing)

    def robotPeriodic(self) -> None:                                      # runs 50 times a second, in every mode
        for name, module in self.drivetrain.modules:                      # repeat for each of the 4 modules
            self.motorSensor[name].update(module.steeringDeg())           # motor sensor, degrees
            self.encoder[name].update(module.absoluteDeg())               # encoder, degrees

    def testSteering(self, name: str) -> commands2.Command:               # a command: test one module
        module = self.drivetrain.module(name)                             # one module by name, e.g. "FL"
        return commands2.FunctionalCommand(                               # a command built from 4 functions:
            lambda: self.startTest(name),                                 # 1. at the start
            lambda: module.spinSteering(TEST_VOLTS),                      # 2. every loop: fixed volts, no PID
            lambda interrupted: self.endTest(name),                       # 3. at the end
            lambda: False,                                                # 4. finished? (never: the timeout ends it)
            self.drivetrain,                                              # it uses the drivetrain
        ).withTimeout(TEST_SECONDS)                                       # stop after TEST_SECONDS

    def startTest(self, name: str) -> None:                               # runs at the start of a test
        for _, module in self.drivetrain.modules:                         # repeat for each of the 4 modules
            module.stopMotors()                                           # everything off, so nothing else moves
        sensors = self.motorSensor.items()                                # (name, tracker) for each module
        self.motorStart = {n: t.total for n, t in sensors}                # where each motor sensor started
        encoders = self.encoder.items()                                   # same for the encoders
        self.encoderStart = {n: t.total for n, t in encoders}             # where each encoder started
        print(f"---- {name}: +{TEST_VOLTS} V. WATCH the wheel. ----")

    def endTest(self, name: str) -> None:                                 # runs at the end of a test: the report
        module = self.drivetrain.module(name)                             # one module by name, e.g. "FL"
        module.stopMotors()                                               # both of its motors off
        now = self.motorSensor[name].total                                # motor sensor's total now
        motor = now - self.motorStart[name]                               # how far the motor turned
        start = self.encoderStart                                         # where each encoder started
        enc = {n: t.total - start[n] for n, t in self.encoder.items()}    # how far each encoder turned
        mine = enc[name]                                                  # how far THIS module's encoder turned
        setting = SETTING_NAMES[name]                                     # e.g. "FRONT_LEFT"
        print(f"  motor sensor turned {motor:+.0f} deg")                  # + means counterclockwise
        print(f"  encoder turned {mine:+.0f} deg")

        if abs(motor) < 10:                                               # 1. did the motor turn at all?
            print("  PROBLEM: the motor hardly turned. Check")
            print(f"  {setting} turn_can, the motor wires,")
            print("  and that nothing is jammed.")
            return                                                        # stop here: nothing else to check

        if abs(mine) < 0.3 * abs(motor):                                  # 2. did THIS module's encoder see it?
            others = [n for n, d in enc.items()                           # did another module's
                      if n != name and abs(d) > 0.3 * abs(motor)]         # encoder see it instead?
            if others:                                                    # yes: the plugs are probably swapped
                other = self.drivetrain.module(others[0])                 # the module whose encoder moved
                port = other.turningEncoder.dioChannel                    # its DIO port
                print(f"  PROBLEM: {name}'s encoder didn't move,")
                print(f"  but {others[0]}'s did (DIO {port}).")
                print(f"  If the {name} wheel turned: set")
                print(f"  encoder_dio={port} for {setting}")
                print(f"  (and fix {others[0]} to match).")
                print("  If a DIFFERENT wheel turned, the")
                print("  turn_can numbers are mixed up instead.")
            else:                                                         # no encoder moved at all
                print(f"  PROBLEM: {name}'s encoder didn't move.")
                print("  Check its 3 wires and DIO number, and")
                print("  that it is on the steering shaft.")
            return

        if (motor > 0) == (mine > 0):                                     # 3. both turned the same way?
            print("  Motor and encoder agree. LOOK at the wheel:")
            print("    counterclockwise -> OK")
            print("    clockwise -> both are backwards:")
            print(f"      flip turn_motor_bottom for {setting}")
            print(f"      AND flip encoder_reversed for {setting}")
        else:                                                             # they turned opposite ways
            print("  PROBLEM: motor and encoder DISAGREE")
            print("  (this makes the wheel fight itself). LOOK:")
            print("    counterclockwise -> encoder backwards:")
            print(f"      flip encoder_reversed for {setting}")
            print("    clockwise -> motor backwards:")
            print(f"      flip turn_motor_bottom for {setting}")
        print("  (flip: True becomes False, False becomes True.")
        print("  Deploy and test again after a change.)")

    def makeAuto(self):                                                   # builds what autonomous will do
        steps = []                                                        # a list of commands to run in order
        for name in ("FL", "FR", "RL", "RR"):                             # each module in turn:
            steps.append(self.testSteering(name))                         # test it
            steps.append(commands2.WaitCommand(0.5))                      # then pause half a second
        return commands2.SequentialCommandGroup(*steps)                   # * = use the whole list

    def autonomousInit(self) -> None:                                     # runs once when enabled in Autonomous
        self.autoCommand = self.makeAuto()                                # build the auto (makeAuto, above)
        if self.autoCommand is not None:                                  # if there is an auto...
            scheduler = commands2.CommandScheduler.getInstance()          # the part that runs commands
            scheduler.schedule(self.autoCommand)                          # start the auto

    def teleopInit(self) -> None:                                         # runs once when enabled in Teleoperated
        if self.autoCommand is not None:                                  # if an auto is still running...
            self.autoCommand.cancel()                                     # stop the auto: the driver takes over
        for _, module in self.drivetrain.modules:                         # repeat for each of the 4 modules
            module.stopMotors()                                           # both of its motors off

    def testInit(self) -> None:                                           # runs once when enabled in Test
        commands2.CommandScheduler.getInstance().cancelAll()              # stop every command that is running
