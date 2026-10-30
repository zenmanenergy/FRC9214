#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
# Python (RobotPy 2026) port of REV's EasySwerve-Java-Template v2026.0
# (subsystems/DriveSubsystem.java).
#

import math
import typing

import navx
import wpilib
from commands2 import Subsystem
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModuleState,
)

from .constants import DriveConstants
from ..tuning import calibration
from ..tuning import gains as tunedGains
from .easyswervemodule import EasySwerveModule


class DriveSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Gains saved by autotune (REV defaults if autotune has never run)
        self.gains = tunedGains.load()
        print(f"[drive] gains: {self.gains.summary()}")

        # Steering encoder zeros (only used when encoders are on the roboRIO):
        # the ones saved on the roboRIO by the Calibrate button, else the
        # backup STEERING_ZEROS_DEG in robot_settings.py
        saved = calibration.loadOffsets()
        backup = DriveConstants.kSteeringEncoderOffsetsRad
        if saved is not None:
            offsets, source = saved, "saved on the roboRIO"
        elif backup is not None:
            offsets, source = backup, "backup from robot_settings.py (saved zeros missing)"
        else:
            offsets, source = {}, "NOT ZEROED YET: press the Calibrate button (examples/32_zerosteering)"
        print(f"[drive] steering encoder zeros: {source}")

        # Create EasySwerveModules
        self.frontLeft = EasySwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset,
            DriveConstants.kFrontLeftDrivingMotorOnBottom,
            DriveConstants.kFrontLeftTurningMotorOnBottom,
            self.gains,
            name="FL",
            steeringEncoderDio=DriveConstants.kFrontLeftSteeringEncoderDio,
            steeringEncoderOffset=offsets.get("FL", 0.0),
        )

        self.frontRight = EasySwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset,
            DriveConstants.kFrontRightDrivingMotorOnBottom,
            DriveConstants.kFrontRightTurningMotorOnBottom,
            self.gains,
            name="FR",
            steeringEncoderDio=DriveConstants.kFrontRightSteeringEncoderDio,
            steeringEncoderOffset=offsets.get("FR", 0.0),
        )

        self.rearLeft = EasySwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kRearLeftChassisAngularOffset,
            DriveConstants.kRearLeftDrivingMotorOnBottom,
            DriveConstants.kRearLeftTurningMotorOnBottom,
            self.gains,
            name="RL",
            steeringEncoderDio=DriveConstants.kRearLeftSteeringEncoderDio,
            steeringEncoderOffset=offsets.get("RL", 0.0),
        )

        self.rearRight = EasySwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kRearRightChassisAngularOffset,
            DriveConstants.kRearRightDrivingMotorOnBottom,
            DriveConstants.kRearRightTurningMotorOnBottom,
            self.gains,
            name="RR",
            steeringEncoderDio=DriveConstants.kRearRightSteeringEncoderDio,
            steeringEncoderOffset=offsets.get("RR", 0.0),
        )

        # The gyro: navX2 on the roboRIO's MXP port (SPI) or a USB port, as set
        # by GYRO in robot_settings.py. (REV's template uses an ADIS16470.)
        port = str(DriveConstants.kGyroPort).lower()
        if port not in ("mxp", "usb"):
            raise ValueError(f'GYRO in robot_settings.py must be "mxp" or "usb", got {port!r}')
        comType = navx.AHRS.NavXComType.kUSB1 if port == "usb" else navx.AHRS.NavXComType.kMXP_SPI
        self.gyro = navx.AHRS(comType)

        # Robot position on the field. A pose estimator (Kalman filter) rather
        # than plain odometry: every loop it combines the drive encoders, the
        # steering angles and the navX heading; when a camera position arrives
        # (addVisionMeasurement) it blends that in, compensating for the
        # camera's delay. With no camera it behaves exactly like odometry.
        self.poseEstimator = SwerveDrive4PoseEstimator(
            DriveConstants.kDriveKinematics,
            self._gyroRotation(),
            self._modulePositions(),
            Pose2d(),
            DriveConstants.kOdometryStdDevs,
            DriveConstants.kDefaultVisionStdDevs,
        )
        self.odometry = self.poseEstimator  # old name, kept for existing code
        self.visionMeasurementCount = 0
        self.poseResetCount = 0  # bumps on every resetOdometry (the sim watches it)

        # REV's Java template also calls HAL.report() here for FIRST's anonymous
        # usage statistics. It has no effect on robot behavior, so it's omitted.

    def periodic(self) -> None:
        # Update the position from wheels and gyro every loop
        self.poseEstimator.update(self._gyroRotation(), self._modulePositions())

        # Keep the steering sensors synced to the absolute encoders (roboRIO
        # wiring), and each steering friction push pointed the right way
        enabled = wpilib.DriverStation.isEnabled()
        now = wpilib.Timer.getFPGATimestamp()
        for _, m in self.modules:
            if m.turningVoltage is None:
                m.turningEncoder.sync(enabled, now)
            m.updateSteeringFeedforward()
        self._publishSteering()

    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot (wheels, gyro and,
        when available, cameras combined)."""
        return self.poseEstimator.getEstimatedPosition()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the estimated position to the specified pose."""
        self.poseEstimator.resetPosition(self._gyroRotation(), self._modulePositions(), pose)
        self.poseResetCount += 1

    def addVisionMeasurement(
        self,
        pose: Pose2d,
        timestampSeconds: float,
        stdDevs: typing.Optional[typing.Tuple[float, float, float]] = None,
    ) -> None:
        """Blend a camera position estimate into the robot position.

        :param pose: where the camera says the robot is (field meters, blue origin)
        :param timestampSeconds: when the picture was taken (FPGA time, as
                                 PhotonVision reports it)
        :param stdDevs: how much to trust it (x m, y m, heading rad; bigger =
                        trust less); None uses DriveConstants.kDefaultVisionStdDevs
        """
        if stdDevs is None:
            self.poseEstimator.addVisionMeasurement(pose, timestampSeconds)
        else:
            self.poseEstimator.addVisionMeasurement(pose, timestampSeconds, stdDevs)
        self.visionMeasurementCount += 1

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward), -1 to 1.
        :param ySpeed:        Speed of the robot in the y direction (sideways), -1 to 1.
        :param rot:           Angular rate of the robot, -1 to 1.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        """
        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = rot * DriveConstants.kMaxAngularSpeed

        if fieldRelative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered, self._gyroRotation()
            )
        else:
            speeds = ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)

        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds)
        self.setModuleStates(swerveModuleStates)

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        if not self.steeringEncodersReady():
            self._stopNotReady()
            return
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(self, desiredStates: typing.Sequence[SwerveModuleState]) -> None:
        """Sets the swerve ModuleStates (front left, front right, rear left, rear right)."""
        if not self.steeringEncodersReady():
            # Steering angle unknown (encoder not read yet or unplugged): driving
            # now would point wheels in random directions, so stay still.
            self._stopNotReady()
            return
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.frontLeft.resetEncoders()
        self.rearLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.rearRight.resetEncoders()

    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def getHeading(self) -> float:
        """Returns the robot's heading in degrees, from -180 to 180."""
        return self._gyroRotation().degrees()

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot, in degrees per second."""
        # The navX reports clockwise-positive rates; WPILib expects
        # counterclockwise-positive, so flip the sign.
        rate = -self.gyro.getRate()
        return rate * (-1.0 if DriveConstants.kGyroReversed else 1.0)

    # ---- used by autotune ----------------------------------------------------

    @property
    def modules(self):
        """(name, module) pairs in FL, FR, RL, RR order."""
        return (
            ("FL", self.frontLeft),
            ("FR", self.frontRight),
            ("RL", self.rearLeft),
            ("RR", self.rearRight),
        )

    def _stopNotReady(self) -> None:
        for _, m in self.modules:
            m.setDrivingVoltage(0.0)
        now = wpilib.Timer.getFPGATimestamp()
        if now - getattr(self, "_lastNotReadyWarn", -1e9) > 2.0:
            self._lastNotReadyWarn = now
            missing = [n for n, m in self.modules if not m.turningEncoder.synced]
            wpilib.reportWarning(
                f"Drive held still: steering encoders not read yet for {', '.join(missing)}"
            )

    def steeringEncodersReady(self) -> bool:
        """True when every module knows its true steering angle."""
        return all(m.turningEncoder.synced for _, m in self.modules)

    def zeroSteeringEncoders(self) -> typing.Optional[str]:
        """roboRIO wiring only: make the current wheel angles the zeros (hex keys
        in, robot disabled), save them, and apply them. Returns a message."""
        if any(m.turningEncoder.location != "rio" for _, m in self.modules):
            return (
                "steering encoders are on the SPARKs; zero them with REV Hardware "
                "Client (Absolute Encoder utility) instead"
            )
        if wpilib.DriverStation.isEnabled():
            return "disable the robot first (hex keys must be in and wheels still)"
        raw = {}
        for name, m in self.modules:
            r = m.turningEncoder.rawAbsolute()
            if r is None:
                return f"encoder {name} on DIO {m.turningEncoder.dioChannel} has no signal; nothing saved"
            raw[name] = r
        path = calibration.saveOffsets(raw)
        for name, m in self.modules:
            m.turningEncoder.setOffset(raw[name])
        backup = ", ".join(f'"{n}": {math.degrees(v):.2f}' for n, v in raw.items())
        print(f"[drive] steering zeros saved to {path}")
        print("[drive] backup: paste this line into robot_settings.py:")
        print(f"STEERING_ZEROS_DEG = {{{backup}}}")
        return f"zeroed and saved ({path})"

    def applyGains(self, gains: tunedGains.TunedGains) -> None:
        """Send new gains to all 8 SPARKs right away."""
        self.gains = gains
        for _, m in self.modules:
            m.applyGains(gains)

    def _publishSteering(self) -> None:
        sd = wpilib.SmartDashboard
        sd.putBoolean("Drive/Steering encoders ready", self.steeringEncodersReady())
        for name, m in self.modules:
            enc = m.turningEncoder
            sd.putNumber(f"Drive/{name} steering deg", math.degrees(m.getState().angle.radians()))
            if enc.location == "rio":
                a = enc.absoluteAngle()
                sd.putNumber(f"Drive/{name} absolute deg", math.degrees(a) if a is not None else -1.0)
                sd.putNumber(f"Drive/{name} resyncs", enc.resyncCount)

    # ---- helpers -------------------------------------------------------------

    def _gyroRotation(self) -> Rotation2d:
        # getRotation2d() already converts the navX's clockwise-positive angle
        # to WPILib's counterclockwise-positive convention. Do not use
        # getAngle() or getYaw() directly here: they are clockwise-positive and
        # would make field-relative driving and odometry turn the wrong way.
        return self.gyro.getRotation2d()

    def _modulePositions(self):
        return (
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.rearLeft.getPosition(),
            self.rearRight.getPosition(),
        )
