#
# ADVANCED SETTINGS (coaches). Students: edit robot_settings.py instead.
#
# Everything specific to one robot (CAN IDs, ports, sizes) is read from
# robot_settings.py in the project folder. This file holds the EasySwerve
# module facts and tuning knobs that rarely change.
#
# Based on REV's EasySwerve-Java-Template v2026.0 (Constants.java).
# All lengths are in meters.
#

import math

import robot_settings as RS

from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics


class NeoMotorConstants:
    # NEO and NEO 2.0 share the same free speed (5676 RPM at 12 V), so REV's
    # NEO numbers apply unchanged to NEO 2.0.
    kFreeSpeedRpm = 5676


class ModuleConstants:
    # Where the steering Through Bore Encoders are wired:
    #   "rio"   = roboRIO DIO ports, 3 wires each (signal, 5 V, ground). Set the
    #             DIO channel for each module in DriveConstants.
    #   "spark" = the turning SPARK MAX data port (REV's original setup)
    kSteeringEncoderLocation = RS.STEERING_ENCODERS

    # Which REV Through Bore Encoder is used for steering.
    #   1 = original Through Bore Encoder (REV-11-1271, discontinued)
    #   2 = Through Bore Encoder V2 (REV-11-3174, ships with EasySwerve kits)
    # Both have the same 8192 counts/rev quadrature output. They differ in the
    # absolute (pulse width) signal used for steering: V1 sends a 1025 us
    # period (1 to 1024 us pulse, 10-bit), V2 a 1000 us period (about 3.9 to
    # 998 us pulse, 12-bit). The wrong setting makes steering angles slightly
    # wrong, so match it to your hardware.
    kThroughBoreEncoderVersion = RS.ENCODER_VERSION

    # Steering gear ratio: steering motor turns per module turn (REV spec).
    # Used when the encoders are on the roboRIO (the SPARK then steers with the
    # NEO's own sensor, kept in sync with the absolute encoder).
    kSteeringReduction = 20.0

    # With encoders on the roboRIO: re-sync the NEO steering sensor to the
    # absolute encoder when they disagree by more than this while the wheel is
    # still (always re-synced while disabled).
    kSteeringResyncThresholdDeg = 1.0

    # Voltage compensation: the SPARKs scale their output so a given command
    # means the same volts whatever the battery is doing (fresh, tired or
    # sagging under load). Gains are computed for this voltage.
    kNominalVoltage = 12.0

    # Drive speed measurement (NEO built-in sensor). REV's default averages over
    # a long window (32 ms period, depth 8), so the reading lags real speed by
    # roughly 130 ms and limits how strong the speed loop can be. 16 ms with
    # depth 2 cuts that to roughly 15 ms. REV allows periods of 8 to 64 ms and
    # depths of 1, 2, 4 or 8. Rerun drive autotune after changing these.
    kDriveVelocityMeasurementPeriodMs = 16
    kDriveVelocityAverageDepth = 2

    # Steering friction feedforward: when the wheel is more than this far from
    # its target, a small push (sized by autotune from measured friction) is
    # added in the direction of the error. Inside it, P alone holds position.
    kSteerFrictionDeadbandDeg = 0.25

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = getattr(RS, "WHEEL_DIAMETER_M", 0.1016)
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # Drive gear ratio 6.3 : 1, from REV's EasySwerve specifications. (REV's
    # Java template computes 7.5 : 1 from tooth counts that don't match the
    # module; 6.3 : 1 agrees with REV's published top speed of 4.79 m/s for
    # NEO 2.0 and with the template's own 4.8 m/s max speed.)
    kDrivingMotorReduction = 6.3
    # Wheel free speed in meters per second (REV names it "Rps" but the
    # circumference factor makes it m/s). About 4.79 m/s.
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction


class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds. Set to the wheel free speed
    # (about 4.79 m/s, REV's template uses 4.8); lower it for headroom.
    kMaxSpeedMetersPerSecond = min(RS.MAX_SPEED_MPS, ModuleConstants.kDriveWheelFreeSpeedRps)
    kMaxAngularSpeed = math.radians(RS.MAX_TURN_DEG_PER_S)  # radians per second

    # Chassis configuration (REV default is a 0.6858 m square; measure yours)
    kTrackWidth = RS.TRACK_WIDTH_M
    # Distance between centers of right and left wheels on robot
    kWheelBase = RS.WHEEL_BASE_M
    # Distance between front and back wheels on robot
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    )

    # Angular offsets of the modules relative to the chassis in radians
    kEasySwerveAngularOffsetCompensation = math.pi / 4
    kFrontLeftChassisAngularOffset = (-math.pi / 2) + kEasySwerveAngularOffsetCompensation
    kFrontRightChassisAngularOffset = 0 + kEasySwerveAngularOffsetCompensation
    kRearLeftChassisAngularOffset = math.pi + kEasySwerveAngularOffsetCompensation
    kRearRightChassisAngularOffset = (math.pi / 2) + kEasySwerveAngularOffsetCompensation

    # The EasySwerve module allows installation of the motors either on top or
    # bottom of the module. These constants configure the location of the motors.
    # The default configuration is with both motors on the bottom of the module.
    kFrontLeftDrivingMotorOnBottom = RS.FRONT_LEFT.drive_motor_bottom
    kRearLeftDrivingMotorOnBottom = RS.BACK_LEFT.drive_motor_bottom
    kFrontRightDrivingMotorOnBottom = RS.FRONT_RIGHT.drive_motor_bottom
    kRearRightDrivingMotorOnBottom = RS.BACK_RIGHT.drive_motor_bottom

    kFrontLeftTurningMotorOnBottom = RS.FRONT_LEFT.turn_motor_bottom
    kRearLeftTurningMotorOnBottom = RS.BACK_LEFT.turn_motor_bottom
    kFrontRightTurningMotorOnBottom = RS.FRONT_RIGHT.turn_motor_bottom
    kRearRightTurningMotorOnBottom = RS.BACK_RIGHT.turn_motor_bottom

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = RS.FRONT_LEFT.drive_can
    kRearLeftDrivingCanId = RS.BACK_LEFT.drive_can
    kFrontRightDrivingCanId = RS.FRONT_RIGHT.drive_can
    kRearRightDrivingCanId = RS.BACK_RIGHT.drive_can

    kFrontLeftTurningCanId = RS.FRONT_LEFT.turn_can
    kRearLeftTurningCanId = RS.BACK_LEFT.turn_can
    kFrontRightTurningCanId = RS.FRONT_RIGHT.turn_can
    kRearRightTurningCanId = RS.BACK_RIGHT.turn_can

    # roboRIO DIO channel of each module's steering Through Bore Encoder
    # (used when ModuleConstants.kSteeringEncoderLocation = "rio")
    kFrontLeftSteeringEncoderDio = RS.FRONT_LEFT.encoder_dio
    kFrontRightSteeringEncoderDio = RS.FRONT_RIGHT.encoder_dio
    kRearLeftSteeringEncoderDio = RS.BACK_LEFT.encoder_dio
    kRearRightSteeringEncoderDio = RS.BACK_RIGHT.encoder_dio

    # Backup steering zeros (radians), from STEERING_ZEROS_DEG in
    # robot_settings.py; None if there's no backup. Used only when the zeros
    # saved on the roboRIO are missing.
    _zerosDeg = getattr(RS, "STEERING_ZEROS_DEG", None)
    kSteeringEncoderOffsetsRad = (
        {name: math.radians(deg) for name, deg in _zerosDeg.items()} if _zerosDeg else None
    )

    kGyroReversed = False
    kGyroPort = RS.GYRO  # "mxp" or "usb"

    # Position tracking (pose estimator) trust levels, as standard deviations:
    # (x m, y m, heading rad). Smaller = trusted more. Wheels and gyro are
    # trusted closely from moment to moment; heading from the navX very much so.
    kOdometryStdDevs = (0.05, 0.05, 0.01)
    # Default trust for a camera position when the caller gives none. The
    # vision add-on normally sets its own, per measurement.
    kDefaultVisionStdDevs = (0.9, 0.9, 0.9)


class OIConstants:
    kDriverControllerPort = RS.CONTROLLER_PORT
    kDriveDeadband = 0.1


class AutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1


class AutotuneConstants:
    """Settings for tuning/autotune.py. The defaults suit an EasySwerve robot;
    change the distance if you have less room."""

    # Only allow autotune in Test mode (it moves the robot by itself).
    kRequireTestMode = True

    # Steering test (robot stays in place; wheels spin on their pivots)
    kSteerRampVoltsPerSecond = 0.5  # slow ramp, finds friction and speed
    kSteerRampSeconds = 8.0
    kSteerStepVolts = 3.0  # sudden step, finds acceleration
    kSteerStepSeconds = 1.0
    kSteerDampingRatio = 0.75  # model-based starting point for steering P
    # Steering friction feedforward as a fraction of measured friction (kS).
    # Below 1.0 so the push alone can never move the wheel; P finishes the job.
    kSteerFrictionFraction = 0.8

    # Steering check-and-adjust steps
    kSteerVerifyStepDegrees = 60.0
    kSteerVerifyMaxIterations = 6
    kSteerOvershootMax = 0.10  # more than 10 % overshoot: lower P
    kSteerOvershootMin = 0.03  # less than 3 %: try a higher P

    # Drive test (robot drives straight forward, then back to where it started)
    kDriveMaxDistanceM = 2.0  # clear floor needed in front of the robot
    kDriveRampVoltsPerSecond = 0.75
    kDriveRampMaxSeconds = 7.0
    kDriveStepVolts = 5.0
    kDriveStepMaxSeconds = 2.0
    # Drive speed loop response: closed-loop time constant as a multiple of the
    # measured speed-reading lag. 1.0 = fastest (about 8 % overshoot),
    # 1.5 = default (about 4 %), 2.0 = gentlest (about 1 %).
    kDriveSmoothing = 1.5

    # Pause between tests (s)
    kRestSeconds = 1.0

    # Warn if a fitted value is this far (fraction) from the median of the 4 modules
    kOutlierFraction = 0.30
