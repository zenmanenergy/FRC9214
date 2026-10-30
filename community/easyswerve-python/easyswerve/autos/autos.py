"""
Autos: autonomous routines drawn in the PathPlanner app.

    autos = Autos(drivetrain)
    autos.addCommand("shoot", shooter.shoot())    # usable by name in the app
    command = autos.selected()                    # the auto picked on the dashboard

Every auto saved in deploy/pathplanner/autos/ appears in the dashboard's
"Auto" chooser automatically.
"""

import typing

import commands2
import wpilib
from pathplannerlib.path import PathConstraints
from wpimath.geometry import Pose2d, Rotation2d

from ..drivetrain.constants import DriveConstants
from . import robotconfig
from .planner import EasySwervePathPlanner

# In the simulator, clicking Autonomous runs this auto so something happens
# without opening a dashboard. On the real robot the default is "Do nothing".
SIM_DEFAULT_AUTO = "Test S-Curve"


class Autos:
    def __init__(self, drivetrain, defaultAuto: typing.Optional[str] = None) -> None:
        """
        :param drivetrain: the Drivetrain
        :param defaultAuto: auto selected when the robot starts (name as in the
                            PathPlanner app); None = "Do nothing" on the robot
        """
        self.drivetrain = drivetrain
        try:
            config = robotconfig.fromApp()
        except FileNotFoundError as e:
            wpilib.reportWarning(f"{e} Using built-in EasySwerve values for now.")
            config = robotconfig.fromValues(
                massKg=50.0,
                moiKgM2=robotconfig.boxMOI(50.0, 0.85, 0.85),
                trackWidthM=DriveConstants.kTrackWidth,
                wheelBaseM=DriveConstants.kWheelBase,
            )
        self.planner = EasySwervePathPlanner(
            drivetrain, DriveConstants.kDriveKinematics, robotConfig=config
        )

        if defaultAuto is None and wpilib.RobotBase.isSimulation():
            defaultAuto = SIM_DEFAULT_AUTO
        self.chooser = self.planner.buildAutoChooser(defaultAutoName=defaultAuto or "")

        # Show the planned path on the dashboard's Robot Position map
        field = getattr(drivetrain, "field", None)
        if field is not None:
            EasySwervePathPlanner.logToField(field)

    def addCommand(self, name: str, command: commands2.Command) -> None:
        """Let autos (and path event markers) run this command by name."""
        EasySwervePathPlanner.registerNamedCommand(name, command)

    def addAuto(self, name: str, makeCommand: typing.Callable[[], commands2.Command]) -> None:
        """Add an auto written in code to the dashboard chooser.

        :param makeCommand: a function that builds the auto's command
        """
        self.chooser.addOption(name, makeCommand)

    def selected(self) -> commands2.Command:
        """The auto picked in the dashboard's Auto chooser."""
        return self.chooser.getSelected()()

    def auto(self, autoName: str) -> commands2.Command:
        """One auto from the PathPlanner app, by its name."""
        return self.planner.buildAuto(autoName)

    def drivePath(self, pathName: str) -> commands2.Command:
        """Set the robot's position to the start of a path from the app, then
        drive the path. For autos: put the real robot at the path's start."""
        return commands2.SequentialCommandGroup(
            self.planner.startAt(pathName), self.planner.followPath(pathName)
        )

    def goToPath(self, pathName: str, maxSpeedMps: float = 3.0) -> commands2.Command:
        """Drive by itself to the start of a path from wherever the robot is,
        then drive the path. Good on a button: hold to go, let go to stop."""
        constraints = PathConstraints(maxSpeedMps, 3.0, 3.14, 6.28)
        return commands2.DeferredCommand(
            lambda: self.planner.pathfindThenFollowPath(pathName, constraints),
            self.drivetrain,
        )

    def followPath(self, pathName: str) -> commands2.Command:
        """Drive one path from the app starting from wherever the robot is now
        (no position reset). Usually you want drivePath or goToPath instead."""
        return commands2.DeferredCommand(
            lambda: self.planner.followPath(pathName), self.drivetrain
        )

    def driveTo(
        self, x: float, y: float, headingDeg: float, maxSpeedMps: float = 3.0
    ) -> commands2.Command:
        """Drive by itself to a spot on the field (meters from the blue
        corner), steering around obstacles. Good on a button: hold to go,
        let go to stop."""
        constraints = PathConstraints(maxSpeedMps, 3.0, 3.14, 6.28)
        target = Pose2d(x, y, Rotation2d.fromDegrees(headingDeg))
        return commands2.DeferredCommand(
            lambda: self.planner.pathfindToPose(target, constraints), self.drivetrain
        )
