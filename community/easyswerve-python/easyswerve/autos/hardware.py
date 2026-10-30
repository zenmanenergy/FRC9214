"""
Physical facts about the REV 4in EasySwerve Module, used to build a PathPlanner
RobotConfig in code when you don't want to rely on the PathPlanner app's
settings file.

These are properties of the module itself, not of any one robot, so this
package keeps its own copy rather than importing the drivetrain's constants.
"""

import math

from wpimath.system.plant import DCMotor

# Wheel: 4in EasySwerve wheel = 101.6 mm diameter
WHEEL_DIAMETER_M = 0.1016
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2

# Drive gearing 6.3 : 1 (REV's EasySwerve specifications; matches REV's
# published 4.79 m/s top speed with NEO 2.0)
DRIVE_REDUCTION = 6.3

# NEO and NEO 2.0 share the same free speed: 5676 RPM at 12 V
NEO_FREE_SPEED_RPM = 5676

# Theoretical wheel free speed, about 4.79 m/s
WHEEL_FREE_SPEED_MPS = (NEO_FREE_SPEED_RPM / 60) * (WHEEL_DIAMETER_M * math.pi) / DRIVE_REDUCTION

# Drive current limit used by the EasySwerve drivetrain template (configs.py)
DRIVE_CURRENT_LIMIT_A = 50


def driveMotor() -> DCMotor:
    """One NEO (or NEO 2.0) driving one wheel through the EasySwerve reduction.

    WPILib has no separate NEO 2.0 model; its electrical constants match the
    original NEO closely enough for path planning.
    """
    return DCMotor.NEO(1).withReduction(DRIVE_REDUCTION)
