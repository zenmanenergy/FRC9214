import math
import wpimath.geometry as geo

# ==================== Robot Dimensions ====================

# TODO (RB, 3/16/26): Swap out these example values for real robot dimensions

# Distance between front and back wheels (in meters)
wheelBase = 0.5

# Distance between front wheels (in meters)
trackWidth = 0.5

# This dictionary defines swerve module positions relative to robot center
modulePositions = {
    'frontLeft': geo.Translation2d(wheelBase / 2, trackWidth / 2),
    'frontRight': geo.Translation2d(wheelBase / 2, -trackWidth / 2),
    'backLeft': geo.Translation2d(-wheelBase / 2, trackWidth / 2),
    'backRight': geo.Translation2d(-wheelBase / 2, -trackWidth / 2)
}

# ==================== Speed Scalers ====================

# Controls max speed of EasySwerve motors, from 4 to 5 (meters per second)
kMaxSpeed = 4.5


# Controls rotational speed of EasySwerve motors, from 0 to 2 (radians per second)

# One rotation per second
kMaxAngularSpeed = 2 * math.pi

# Controls how quickly the robot can accelerate to max speed

# 3 = 1 second to from 0 to max speed, 6 = 0.5 seconds, etc
kTranslationSlew = 3
kRotationSlew = 3

# ==================== Swerve Module Offsets ====================

# TODO (RB, 3/16/26): Swap out these example values for real robot module offsets

# This dictionary ndividually defines what angle is stright for each module (radians)
moduleOffsets = {
    'frontLeft': -2.31,
    'frontRight': 1.78,
    'backLeft': 0.92,
    'backRight': -0.55
}

# ==================== CAN IDs ====================

# Front left swerve module
SWERVE_FRONT_LEFT_DRIVE = 15
SWERVE_FRONT_LEFT_TURN = 13

# Front right swerve module
SWERVE_FRONT_RIGHT_DRIVE = 3
SWERVE_FRONT_RIGHT_TURN = 2

# Rear left swerve module
SWERVE_REAR_LEFT_DRIVE = 10
SWERVE_REAR_LEFT_TURN = 9

# Rear right swerve module
SWERVE_REAR_RIGHT_DRIVE = 6
SWERVE_REAR_RIGHT_TURN = 5

# Turret
TURRET_TURN = 11

# Shooter
SHOOTER_SPINDEXER = 8
SHOOTER_UPTAKE = 12
SHOOTER_SHOOTER = 16

# Intake
INTAKE = 4
