"""
EasySwervePathPlanner: connects PathPlanner to an EasySwerve DriveSubsystem.

It needs nothing added to the drivetrain. It only uses these existing
DriveSubsystem methods and attributes:

    getPose()            -> Pose2d
    resetOdometry(pose)
    setModuleStates(states)
    frontLeft / frontRight / rearLeft / rearRight   (each with getState())

plus the drivetrain's SwerveDrive4Kinematics.
"""

import logging
import os
import typing

import commands2
import wpilib
from pathplannerlib.auto import AutoBuilder, NamedCommands
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.logging import PathPlannerLogging
from pathplannerlib.path import PathConstraints, PathPlannerPath
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics

from . import robotconfig as _robotconfig

logger = logging.getLogger("easyswerve_pathplanner")

# Starting gains for path following. These correct position error on top of
# the planned speeds; PathPlanner's own examples use 5.0 for both.
DEFAULT_TRANSLATION_PID = PIDConstants(5.0, 0.0, 0.0)
DEFAULT_ROTATION_PID = PIDConstants(5.0, 0.0, 0.0)


class EasySwervePathPlanner:
    def __init__(
        self,
        drive: commands2.Subsystem,
        kinematics: SwerveDrive4Kinematics,
        *,
        robotConfig: typing.Optional[RobotConfig] = None,
        translationPID: PIDConstants = DEFAULT_TRANSLATION_PID,
        rotationPID: PIDConstants = DEFAULT_ROTATION_PID,
        flipForRedAlliance: bool = True,
    ) -> None:
        """Configure PathPlanner for an EasySwerve DriveSubsystem.

        Create exactly one, after the DriveSubsystem and after registering any
        named commands (see registerNamedCommand).

        :param drive:              the EasySwerve DriveSubsystem
        :param kinematics:         the drivetrain's SwerveDrive4Kinematics
                                   (DriveConstants.kDriveKinematics)
        :param robotConfig:        PathPlanner RobotConfig; default loads the one
                                   saved by the PathPlanner app (see robotconfig.py)
        :param translationPID:     path following X/Y correction gains
        :param rotationPID:        path following heading correction gains
        :param flipForRedAlliance: mirror paths when on the red alliance, so you
                                   draw every path once from the blue side
        """
        self.drive = drive
        self.kinematics = kinematics
        self.flipForRedAlliance = flipForRedAlliance
        self.robotConfig = robotConfig if robotConfig is not None else _robotconfig.fromApp()

        AutoBuilder.configure(
            drive.getPose,
            drive.resetOdometry,
            self.getRobotRelativeSpeeds,
            self.driveRobotRelative,
            PPHolonomicDriveController(translationPID, rotationPID),
            self.robotConfig,
            self.shouldFlipPath,
            drive,
        )

    # ---- functions PathPlanner calls ----------------------------------------

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        """Current robot-relative speeds measured from the 4 module states."""
        d = self.drive
        return self.kinematics.toChassisSpeeds(
            (
                d.frontLeft.getState(),
                d.frontRight.getState(),
                d.rearLeft.getState(),
                d.rearRight.getState(),
            )
        )

    def driveRobotRelative(self, speeds: ChassisSpeeds, feedforwards=None) -> None:
        """Drive at robot-relative speeds (m/s, m/s, rad/s).

        PathPlanner also passes per-module feedforwards; they are ignored
        because the EasySwerve drive SPARKs already run their own velocity
        feedforward (kV) on board.
        """
        discretized = ChassisSpeeds.discretize(speeds, 0.02)
        self.drive.setModuleStates(self.kinematics.toSwerveModuleStates(discretized))

    def shouldFlipPath(self) -> bool:
        if not self.flipForRedAlliance:
            return False
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    # ---- building autos and paths -------------------------------------------

    @staticmethod
    def registerNamedCommand(name: str, command: commands2.Command) -> None:
        """Make a command available to autos and event markers by name.

        Call this BEFORE building autos or the chooser; an auto built earlier
        will silently skip names it doesn't know yet.
        """
        NamedCommands.registerCommand(name, command)

    def buildAuto(self, autoName: str) -> commands2.Command:
        """A full auto drawn in the PathPlanner app, by its file name (no .auto)."""
        return AutoBuilder.buildAuto(autoName)

    def followPath(self, pathName: str) -> commands2.Command:
        """Follow one path drawn in the app, by its file name (no .path)."""
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName))

    def startAt(self, pathName: str) -> commands2.Command:
        """Reset odometry to the starting position of a saved path (mirrored on
        red when flipping is on). Put it first in an auto built in code, the
        way a saved auto's "Reset Odom" does."""
        path = PathPlannerPath.fromPathFile(pathName)
        start = path.getStartingHolonomicPose()
        if start is None:
            raise ValueError(f"path '{pathName}' has no starting rotation set in the app")
        return AutoBuilder.resetOdom(start)

    def pathfindToPose(
        self,
        pose: Pose2d,
        constraints: PathConstraints,
        goalEndVelocityMps: float = 0.0,
    ) -> commands2.Command:
        """Drive on the fly to a field pose (blue-origin coordinates), avoiding
        the obstacles saved in the app's navgrid."""
        return AutoBuilder.pathfindToPose(pose, constraints, goalEndVelocityMps)

    def pathfindThenFollowPath(
        self, pathName: str, constraints: PathConstraints
    ) -> commands2.Command:
        """Drive on the fly to the start of a saved path, then follow it.
        Good for teleop alignment to a scoring location."""
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile(pathName), constraints
        )

    def autoNames(self) -> typing.List[str]:
        """Names of all autos saved in this project, sorted."""
        folder = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
        if not os.path.isdir(folder):
            return []
        return sorted(f[: -len(".auto")] for f in os.listdir(folder) if f.endswith(".auto"))

    def buildAutoChooser(
        self, defaultAutoName: str = "", dashboardKey: typing.Optional[str] = "Auto"
    ) -> wpilib.SendableChooser:
        """A dashboard chooser listing every saved auto.

        Unlike PathPlanner's own builder this does not crash if the project has
        no autos folder yet, and it builds each auto only when it is picked.
        Use chooser.getSelected()() to get the command.

        :param defaultAutoName: auto selected by default ("" means "Do nothing")
        :param dashboardKey:    SmartDashboard key to publish under, or None
        """
        chooser = wpilib.SendableChooser()
        chooser.setDefaultOption("Do nothing", commands2.cmd.none)
        for name in self.autoNames():
            builder = (lambda n=name: self.buildAuto(n))
            if name == defaultAutoName:
                chooser.setDefaultOption(name, builder)
            else:
                chooser.addOption(name, builder)
        if dashboardKey:
            wpilib.SmartDashboard.putData(dashboardKey, chooser)
        return chooser

    # ---- visualization -------------------------------------------------------

    @staticmethod
    def logToField(field: wpilib.Field2d) -> None:
        """Draw the active path and target pose on a Field2d widget."""
        PathPlannerLogging.setLogActivePathCallback(
            lambda poses: field.getObject("path").setPoses(poses)
        )
        PathPlannerLogging.setLogTargetPoseCallback(
            lambda pose: field.getObject("target").setPose(pose)
        )
