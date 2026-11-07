"""
Ways to get the PathPlanner RobotConfig (mass, inertia, wheel grip, module
layout) that PathPlanner needs to plan realistic paths.
"""

import json
import logging
import os
import typing

from pathplannerlib.config import ModuleConfig, RobotConfig
from wpilib import getDeployDirectory
from wpimath.geometry import Translation2d

from . import hardware

logger = logging.getLogger("easyswerve_pathplanner")


def settingsFilePath() -> str:
    """Where the PathPlanner app saves its robot settings in the project."""
    return os.path.join(getDeployDirectory(), "pathplanner", "settings.json")


def fromApp() -> RobotConfig:
    """Load the RobotConfig you entered in the PathPlanner app's Settings page.

    Raises FileNotFoundError with a clear message if the app hasn't saved one
    into this project yet.
    """
    path = settingsFilePath()
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"No PathPlanner settings at {path}. Open this robot project folder in "
            "the PathPlanner app and fill in Settings > Robot Config, or build the "
            "config in code with easyswerve_pathplanner.robotconfig.fromValues()."
        )
    return RobotConfig.fromGUISettings()


def fromValues(
    massKg: float,
    moiKgM2: float,
    trackWidthM: float,
    wheelBaseM: float,
    wheelCOF: float = 1.2,
    maxDriveSpeedMps: typing.Optional[float] = None,
) -> RobotConfig:
    """Build the RobotConfig in code for 4 EasySwerve modules.

    :param massKg:          robot mass with bumpers and battery, kg
    :param moiKgM2:         moment of inertia about the vertical axis, kg m^2
                            (rough estimate for a box robot: massKg * (L^2 + W^2) / 12)
    :param trackWidthM:     left-to-right wheel center distance, m
    :param wheelBaseM:      front-to-back wheel center distance, m
    :param wheelCOF:        wheel-to-carpet friction coefficient (1.2 is typical
                            for treaded wheels; use 1.0 if unsure)
    :param maxDriveSpeedMps: top speed the drive can actually reach on the
                            ground; defaults to the EasySwerve wheel free speed
    """
    module = ModuleConfig(
        hardware.WHEEL_RADIUS_M,
        maxDriveSpeedMps if maxDriveSpeedMps is not None else hardware.WHEEL_FREE_SPEED_MPS,
        wheelCOF,
        hardware.driveMotor(),
        hardware.DRIVE_CURRENT_LIMIT_A,
        1,
    )
    x = wheelBaseM / 2
    y = trackWidthM / 2
    # PathPlanner wants FL, FR, BL, BR, same order as the drivetrain template
    offsets = [
        Translation2d(x, y),
        Translation2d(x, -y),
        Translation2d(-x, y),
        Translation2d(-x, -y),
    ]
    return RobotConfig(massKg, moiKgM2, module, offsets)


def boxMOI(massKg: float, lengthM: float, widthM: float) -> float:
    """Moment of inertia of a uniform rectangular robot, kg m^2. A starting
    estimate; real robots with heavy parts near the edges are higher."""
    return massKg * (lengthM**2 + widthM**2) / 12


def matchRobot(
    config: RobotConfig, trackWidthM: float, wheelBaseM: float, maxSpeedMps: float
) -> RobotConfig:
    """The app's RobotConfig, with the wheel positions and top speed taken from
    robot_settings.py instead (the robot code's own numbers), so PathPlanner
    never plans a path faster than MAX_SPEED_MPS lets the wheels go. Mass,
    inertia, grip and motor stay as entered in the app. If anything goes
    wrong, the app's config is returned unchanged."""
    try:
        m = config.moduleConfig
        module = ModuleConfig(
            m.wheelRadiusMeters,
            min(m.maxDriveVelocityMPS, maxSpeedMps),
            m.wheelCOF,
            m.driveMotor,  # already includes the gear ratio
            m.driveCurrentLimit,
            1,
        )
        x, y = wheelBaseM / 2, trackWidthM / 2
        offsets = [Translation2d(x, y), Translation2d(x, -y), Translation2d(-x, y), Translation2d(-x, -y)]
        return RobotConfig(config.massKG, config.MOI, module, offsets)
    except Exception as e:  # never stop the robot over this
        logger.warning("Could not adjust the PathPlanner robot config (%s); using the app's", e)
        return config


def appMismatches(robotLengthM: float, robotWidthM: float, trackWidthM: float) -> typing.List[str]:
    """Settings in the PathPlanner app (deploy/pathplanner/settings.json) that
    disagree with robot_settings.py. The app uses the robot size to keep paths
    and pathfinding clear of walls, so it should match."""
    try:
        with open(settingsFilePath()) as f:
            app = json.load(f)
    except Exception:
        return []
    out = []
    for key, mine, label in (
        ("robotLength", robotLengthM, "Robot Length (ROBOT_LENGTH_M)"),
        ("robotWidth", robotWidthM, "Robot Width (ROBOT_WIDTH_M)"),
        ("robotTrackwidth", trackWidthM, "Trackwidth (TRACK_WIDTH_M)"),
    ):
        theirs = app.get(key)
        if isinstance(theirs, (int, float)) and abs(theirs - mine) > 0.01:
            out.append(f"{label}: app {theirs:.2f} m, robot_settings.py {mine:.2f} m")
    return out
