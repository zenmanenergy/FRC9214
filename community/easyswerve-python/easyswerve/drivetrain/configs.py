#
# Python (RobotPy 2026) port of REV's EasySwerve-Java-Template v2026.0
# (Configs.java).
#
# REV's Java version keeps one shared static config per motor type and mutates
# its inversion for each module just before applying it. Here each module gets
# its own fresh config object instead, which produces identical SPARK settings
# without the shared-state trap.
#

import math

from rev import (
    AbsoluteEncoderConfig,
    FeedbackSensor,
    SparkBaseConfig,
    SparkMaxConfig,
)

from .constants import ModuleConstants
from ..tuning.gains import TunedGains

# Use module constants to calculate conversion factors and feed forward gain.
kDrivingFactor = (
    ModuleConstants.kWheelDiameterMeters * math.pi / ModuleConstants.kDrivingMotorReduction
)
kTurningFactor = 2 * math.pi

kNominalVoltage = ModuleConstants.kNominalVoltage

# The SPARK runs its closed loop every 1 ms and does not divide by time: each
# loop it adds kI x error to the I term and uses kD x (change in error since
# the last loop). So per-second gains are converted like this:
kSparkLoopSeconds = 0.001
kDrivingVelocityFeedForward = kNominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps


def steeringEncoderLocation() -> str:
    """Where the steering encoders are wired: "rio" or "spark" (constants.py)."""
    loc = str(ModuleConstants.kSteeringEncoderLocation).lower()
    if loc not in ("rio", "spark"):
        raise ValueError(
            f'ModuleConstants.kSteeringEncoderLocation must be "rio" or "spark", got {loc!r}'
        )
    return loc


def throughBoreEncoderPreset(version: int) -> AbsoluteEncoderConfig:
    """REVLib's absolute encoder preset for Through Bore Encoder V1 or V2."""
    if version == 1:
        return AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder()
    if version == 2:
        return AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2()
    raise ValueError(
        f"ModuleConstants.kThroughBoreEncoderVersion must be 1 or 2, got {version!r}"
    )


def drivingConfig(inverted: bool, gains: TunedGains = None) -> SparkMaxConfig:
    """SPARK MAX config for an EasySwerve driving motor (NEO / NEO 2.0).

    :param gains: tuned gains (from autotune); None uses REV's defaults
    """
    gains = gains if gains is not None else TunedGains()
    config = SparkMaxConfig()

    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.smartCurrentLimit(50)  # 70 for SparkFlex
    config.inverted(inverted)
    config.voltageCompensation(kNominalVoltage)

    config.encoder.positionConversionFactor(kDrivingFactor)  # meters
    config.encoder.velocityConversionFactor(kDrivingFactor / 60.0)  # meters per second
    # Shorter speed-averaging window = less lag in the speed reading
    config.encoder.uvwMeasurementPeriod(ModuleConstants.kDriveVelocityMeasurementPeriodMs)
    config.encoder.uvwAverageDepth(ModuleConstants.kDriveVelocityAverageDepth)

    # Fresh position and speed every 20 ms for odometry and autotune
    config.signals.primaryEncoderPositionPeriodMs(20)
    config.signals.primaryEncoderVelocityPeriodMs(20)

    config.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
    # Gains come from autotune (tuning/gains.py); defaults are REV's values.
    config.closedLoop.pid(gains.driveP, 0, 0)
    config.closedLoop.outputRange(-1, 1)
    config.closedLoop.feedForward.kS(gains.driveKS)
    config.closedLoop.feedForward.kV(gains.driveKV)

    return config


def turningConfig(
    inverted: bool,
    gains: TunedGains = None,
    encoderReversed: bool = False,
    fastSignals: bool = False,
) -> SparkMaxConfig:
    """SPARK MAX config for an EasySwerve turning motor (NEO / NEO 2.0) with a
    Through Bore Encoder for steering (on the roboRIO or the SPARK data port,
    see ModuleConstants.kSteeringEncoderLocation).

    :param gains: tuned gains (from autotune); None uses REV's defaults
    :param encoderReversed: encoder on the SPARK counts backwards (encoder_reversed)
    :param fastSignals: send the angle every 5 ms (Ziegler-Nichols tuning)
    """
    gains = gains if gains is not None else TunedGains()
    config = SparkMaxConfig()
    periodMs = 5 if fastSignals else 20

    config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    config.smartCurrentLimit(20)  # 70 for SparkFlex
    config.inverted(inverted)
    config.voltageCompensation(kNominalVoltage)

    if steeringEncoderLocation() == "rio":
        # Absolute encoder is on the roboRIO. The SPARK steers with the NEO's
        # built-in sensor, scaled through the steering gearing to module
        # radians; subsystems/steeringencoder.py keeps it set to the true angle.
        steerFactor = kTurningFactor / ModuleConstants.kSteeringReduction
        config.encoder.positionConversionFactor(steerFactor)  # radians
        config.encoder.velocityConversionFactor(steerFactor / 60.0)  # rad/s
        config.signals.primaryEncoderPositionPeriodMs(periodMs)
        config.signals.primaryEncoderVelocityPeriodMs(20)
        config.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
    else:
        # Do not invert the turning encoder, since the output shaft rotates in
        # the same direction as the steering motor in the EasySwerve Module.
        # (encoder_reversed in robot_settings.py flips it if yours reads backwards)
        config.absoluteEncoder.inverted(encoderReversed)
        config.absoluteEncoder.positionConversionFactor(kTurningFactor)  # radians
        config.absoluteEncoder.velocityConversionFactor(kTurningFactor / 60.0)  # rad/s
        # Pulse-width timing for the encoder model set in constants.py
        # (ModuleConstants.kThroughBoreEncoderVersion).
        config.absoluteEncoder.apply(
            throughBoreEncoderPreset(ModuleConstants.kThroughBoreEncoderVersion)
        )
        # Fresh steering angle every 20 ms for odometry and autotune
        config.signals.absoluteEncoderPositionPeriodMs(periodMs)
        config.signals.absoluteEncoderVelocityPeriodMs(20)
        config.closedLoop.setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder)

    # Gain comes from autotune (tuning/gains.py); default is REV's value.
    config.closedLoop.pid(
        gains.steerP,
        gains.steerI * kSparkLoopSeconds,
        gains.steerD / kSparkLoopSeconds,
    )
    if gains.steerI and gains.steerIZoneDeg > 0:
        config.closedLoop.iZone(math.radians(gains.steerIZoneDeg))
    config.closedLoop.outputRange(-1, 1)
    # Enable PID wrap around for the turning motor. This lets the PID controller
    # go through 0 to reach the setpoint, i.e. going from 350 degrees to 10
    # degrees goes through 0 rather than the long way around.
    config.closedLoop.positionWrappingEnabled(True)
    config.closedLoop.positionWrappingInputRange(0, kTurningFactor)

    return config
