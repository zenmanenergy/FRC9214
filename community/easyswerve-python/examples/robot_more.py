"""
EXAMPLE: robot.py with more features. Copy the parts you want into the
real robot.py.

Shows:
  - adding a mechanism (examples/intake.py) with buttons and auto commands
  - driving by itself to a spot on the field (hold a button)
  - running one PathPlanner path from a button (for testing)
  - an auto written in code instead of the PathPlanner app
"""

import commands2
from commands2.button import CommandXboxController

import robot_settings
from easyswerve import Autos, Drivetrain
from intake import Intake  # after copying examples/intake.py into the project folder


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        # ---- The robot's parts -------------------------------------------------
        self.drivetrain = Drivetrain()
        self.intake = Intake()

        # ---- Driver controller -------------------------------------------------
        self.controller = CommandXboxController(robot_settings.CONTROLLER_PORT)
        self.drivetrain.driveWithController(self.controller)

        self.controller.rightBumper().whileTrue(self.drivetrain.lockWheels())
        self.controller.start().onTrue(self.drivetrain.resetHeading())

        # Intake: hold A to pull in, hold B to spit out
        self.controller.a().whileTrue(self.intake.runIn())
        self.controller.b().whileTrue(self.intake.runOut())

        # ---- Autos ------------------------------------------------------------------
        self.autos = Autos(self.drivetrain)

        # Commands autos can use by name. In the app, put "intake" in a
        # deadline group with a path to run the intake while driving.
        self.autos.addCommand("intake", self.intake.runIn())
        self.autos.addCommand("spitOut", self.intake.runOut().withTimeout(1.0))
        self.autos.addCommand("printHello", commands2.PrintCommand("Hello from auto!"))

        # Hold Y: drive by itself to a spot on the field
        # (x, y in meters from the blue corner, heading in degrees)
        self.controller.y().whileTrue(self.autos.driveTo(2.0, 4.0, 0))

        # Hold X: drive one path from the app (start near the path's beginning)
        self.controller.x().whileTrue(self.autos.followPath("Drive Forward 2m"))

        # An auto written in code; it appears in the Auto chooser too
        self.autos.addAuto("Code: drive, then spit out", self.driveThenSpit)

        self.autoCommand = None

    def driveThenSpit(self) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.autos.followPath("Drive Forward 2m"),
            self.intake.runOut().withTimeout(1.0),
        )

    def autonomousInit(self) -> None:
        self.autoCommand = self.autos.selected()
        commands2.CommandScheduler.getInstance().schedule(self.autoCommand)

    def teleopInit(self) -> None:
        if self.autoCommand is not None:
            self.autoCommand.cancel()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()
