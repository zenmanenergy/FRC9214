#!/usr/bin/env python3
"""
SETUP STEP 8: Tune the drive motors

The drive motors are tuned differently from the steering. A wheel's speed
mostly depends on voltage in a simple, steady way, so instead of
Ziegler-Nichols the robot measures that relationship (feedforward):
  kS  volts just to get the wheel moving (friction)
  kV  volts for each meter per second of speed
then adds a small P to correct what's left. The robot drives itself up to
2 m forward and back to measure them, and saves the result.

On the robot: wheels ON the floor, at least 2.5 m of clear floor IN FRONT of
the robot, someone at the Disable button. Driver Station in Test mode,
then Enable. It takes about 30 seconds. Read the console.

Try it in the simulator: click Test, then watch the terminal. (The simulator's
"true" motor numbers are SIM_DRIVE_KS and SIM_DRIVE_KV in
easyswerve/sim/physics.py: autotune should come close to them.)

To use this example: copy this file over robot.py in the project folder
(the file next to robot_settings.py), then run the simulator.
"""

import commands2

from easyswerve import Drivetrain


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Runs once when the robot turns on: build the robot here."""
        self.drivetrain = Drivetrain()
        self.autoCommand = None
        self.tuneCommand = None

    def testInit(self) -> None:
        """Runs when the robot is enabled in Test mode: start the drive tuning."""
        commands2.CommandScheduler.getInstance().cancelAll()
        # Disabling stops it at once; nothing is saved from a stopped run.
        print("Drive autotune: the robot will drive forward up to 2 m, then back.")
        self.tuneCommand = self.drivetrain.autotuner.driveCommand()
        commands2.CommandScheduler.getInstance().schedule(self.tuneCommand)

    def makeAuto(self):
        """Builds the command that runs in autonomous."""
        return None  # this example runs in Test mode

    def autonomousInit(self) -> None:
        print("This example runs in Test mode: switch the Driver Station to Test and enable.")

    def teleopInit(self) -> None:
        pass
