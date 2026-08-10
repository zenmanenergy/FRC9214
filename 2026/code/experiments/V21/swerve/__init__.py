"""FRC Swerve Drive Library

A complete, production-ready swerve drive system for FRC robots featuring:
- Modular swerve module abstraction
- Field-relative and robot-relative drive modes
- Dead reckoning odometry with IMU sensor fusion
- Adaptive PID tuning with battery voltage correlation
- Autonomous path following with Catmull-Rom splines
- Motor current monitoring for collision detection
- Persistent encoder offset and tuning calibration

Quick Start:
    from swerve import SwerveDrive
    
    swerve = SwerveDrive()
    swerve.drive_swerve(forward=0.5, strafe=0.0, rotate=0.0)
    swerve.odometry.update()

Configuration:
    Edit swerve_config.py to match your robot's dimensions and CAN IDs.

Calibration:
    Run encoder calibration via the tuning system in the main robot code.
    Persistent offsets are stored in encoder_offsets.json on RoboRIO.
"""

__version__ = '1.0.0'
__author__ = 'Team 9214'

from .swerve_config import (
	WHEELS,
	MOTOR_SCALE_ALIGN,
	MOTOR_SCALE_TELEOP,
	ALIGN_TOLERANCE,
	ALIGN_TIMEOUT,
	DRIVE_ANGLE_TOLERANCE,
	ROBOT_TRACKWIDTH_CM,
	ROBOT_WHEELBASE_CM,
	OFFSET_FILE,
)
from .swerve_wheel import SwerveWheel
from .swerve_drive import SwerveDrive
from .swerve_odometry import SwerveOdometry
from .swerve_imu import SwerveIMU
from .pid_controller import PIDController
from .encoder_calibration import EncoderCalibration
from .swerve_tune import SwerveTuner
from .catmull_rom import CatmullRomSpline

__all__ = [
	# Core drive system
	'SwerveDrive',
	'SwerveWheel',
	# Navigation & odometry
	'SwerveOdometry',
	'SwerveIMU',
	'CatmullRomSpline',
	# Control & tuning
	'PIDController',
	'EncoderCalibration',
	'SwerveTuner',
	# Configuration constants
	'WHEELS',
	'MOTOR_SCALE_ALIGN',
	'MOTOR_SCALE_TELEOP',
	'ALIGN_TOLERANCE',
	'ROBOT_TRACKWIDTH_CM',
	'ROBOT_WHEELBASE_CM',
]
