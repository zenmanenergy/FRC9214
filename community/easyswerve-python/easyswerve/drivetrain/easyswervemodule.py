#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
# Python (RobotPy 2026) port of REV's EasySwerve-Java-Template v2026.0
# (subsystems/EasySwerveModule.java).
#

import math
import typing

from rev import ClosedLoopSlot, PersistMode, ResetMode, SparkMax
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from . import configs
from .constants import ModuleConstants
from ..tuning.gains import TunedGains
from .steeringencoder import RioSteeringEncoder, SparkSteeringEncoder


class EasySwerveModule:
    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        chassisAngularOffset: float,
        drivingMotorOnBottom: bool,
        turningMotorOnBottom: bool,
        gains: typing.Optional[TunedGains] = None,
        name: str = "",
        steeringEncoderDio: typing.Optional[int] = None,
        steeringEncoderOffset: float = 0.0,
    ) -> None:
        """Constructs an EasySwerveModule and configures the driving and turning
        motor, encoder, and PID controller. This configuration is specific to the
        REV EasySwerve Module built with NEOs (or NEO 2.0s), SPARK MAXs, and a
        Through Bore Encoder (V1 or V2).
        """
        self.drivingSpark = SparkMax(drivingCANId, SparkMax.MotorType.kBrushless)
        self.turningSpark = SparkMax(turningCANId, SparkMax.MotorType.kBrushless)

        self.name = name
        self.drivingEncoder = self.drivingSpark.getEncoder()
        # Steering angle, wherever the Through Bore Encoder is wired
        # (see subsystems/steeringencoder.py). Radians 0..2pi either way.
        if configs.steeringEncoderLocation() == "rio":
            if steeringEncoderDio is None:
                raise ValueError(
                    f"module {name}: steering encoders are set to the roboRIO but "
                    "no DIO channel was given (DriveConstants.k...SteeringEncoderDio)"
                )
            self.turningEncoder = RioSteeringEncoder(
                self.turningSpark, steeringEncoderDio, steeringEncoderOffset, name
            )
        else:
            self.turningEncoder = SparkSteeringEncoder(self.turningSpark)

        self.drivingClosedLoopController = self.drivingSpark.getClosedLoopController()
        self.turningClosedLoopController = self.turningSpark.getClosedLoopController()

        # Apply the respective configurations to the SPARKS. Reset parameters
        # before applying the configuration to bring the SPARK to a known good
        # state. Persist the settings to the SPARK to avoid losing them on a
        # power cycle.
        self.gains = gains if gains is not None else TunedGains()
        self.drivingInverted = drivingMotorOnBottom
        self.turningInverted = not turningMotorOnBottom

        # While autotune runs a motor open-loop these hold the commanded
        # volts; None means normal closed-loop control.
        self.drivingVoltage: typing.Optional[float] = None
        self.turningVoltage: typing.Optional[float] = None

        # Steering friction push last sent with the steering setpoint (volts),
        # and the setpoint itself (encoder frame, radians)
        self.turningFeedforward = 0.0
        self.turningTarget: typing.Optional[float] = None
        self.steerDeadbandRad = math.radians(ModuleConstants.kSteerFrictionDeadbandDeg)

        self.drivingSpark.configure(
            configs.drivingConfig(self.drivingInverted, self.gains),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        self.turningSpark.configure(
            configs.turningConfig(self.turningInverted, self.gains),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = SwerveModuleState(
            0.0, Rotation2d(self.turningEncoder.getPosition())
        )
        self.drivingEncoder.setPosition(0)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module."""
        # Apply chassis angular offset to the encoder position to get the
        # position relative to the chassis.
        return SwerveModuleState(
            self.drivingEncoder.getVelocity(),
            Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset),
        )

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module."""
        # Apply chassis angular offset to the encoder position to get the
        # position relative to the chassis.
        return SwerveModulePosition(
            self.drivingEncoder.getPosition(),
            Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset),
        )

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """
        # Apply chassis angular offset to the desired state.
        correctedDesiredState = SwerveModuleState(
            desiredState.speed,
            desiredState.angle + Rotation2d(self.chassisAngularOffset),
        )

        # Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(Rotation2d(self.turningEncoder.getPosition()))

        # Command driving and turning SPARKS towards their respective setpoints.
        self.drivingVoltage = None
        self.turningVoltage = None
        self.drivingClosedLoopController.setSetpoint(
            correctedDesiredState.speed, SparkMax.ControlType.kVelocity
        )
        self._steerTo(correctedDesiredState.angle.radians())

        self.desiredState = desiredState

    def resetEncoders(self) -> None:
        """Zeroes all the SwerveModule encoders."""
        self.drivingEncoder.setPosition(0)

    # ---- used by autotune ----------------------------------------------------

    def setDrivingVoltage(self, volts: float) -> None:
        """Run the drive motor open-loop at a voltage (autotune only)."""
        self.drivingVoltage = volts
        self.drivingSpark.setVoltage(volts)

    def setTurningVoltage(self, volts: float) -> None:
        """Run the steering motor open-loop at a voltage (autotune only)."""
        self.turningVoltage = volts
        self.turningSpark.setVoltage(volts)

    def holdAngle(self, robotRelativeAngle: Rotation2d) -> None:
        """Steer to an angle relative to the robot WITHOUT the usual 180 degree
        flip optimization, so every wheel ends up facing the same way."""
        self.turningVoltage = None
        self._steerTo((robotRelativeAngle + Rotation2d(self.chassisAngularOffset)).radians())

    def _frictionPush(self, target: float) -> float:
        error = (target - self.turningEncoder.getPosition() + math.pi) % math.tau - math.pi
        if self.gains.steerKS > 0 and abs(error) > self.steerDeadbandRad:
            return math.copysign(self.gains.steerKS, error)
        return 0.0

    def _steerTo(self, encoderAngle: float) -> None:
        """Send a steering setpoint (encoder frame, radians), adding a friction
        push in the direction of the error when it is outside the deadband."""
        target = encoderAngle % math.tau
        ff = self._frictionPush(target)
        self.turningTarget = target
        self.turningFeedforward = ff
        self.turningClosedLoopController.setSetpoint(
            target, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff
        )

    def updateSteeringFeedforward(self) -> None:
        """Re-aim the friction push at the current error. DriveSubsystem calls
        this every loop, so the push stays correct even if nothing resends the
        steering target (it is only resent when the push actually changes)."""
        if self.turningTarget is None or self.turningVoltage is not None:
            return
        ff = self._frictionPush(self.turningTarget)
        if ff != self.turningFeedforward:
            self.turningFeedforward = ff
            self.turningClosedLoopController.setSetpoint(
                self.turningTarget, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff
            )

    def applyGains(self, gains: TunedGains) -> None:
        """Send new gains to both SPARKs right away. The full config is resent,
        exactly as at startup. Not written to SPARK flash: the saved tuning file
        is reapplied at every startup instead."""
        self.gains = gains
        self.drivingSpark.configure(
            configs.drivingConfig(self.drivingInverted, gains),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )
        self.turningSpark.configure(
            configs.turningConfig(self.turningInverted, gains),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters,
        )
