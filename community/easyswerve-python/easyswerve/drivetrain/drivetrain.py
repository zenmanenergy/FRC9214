"""
Drivetrain: the whole swerve drive, ready to use.

Creating one sets up everything behind the scenes: the four modules, the
gyro, position tracking, cameras (if enabled in robot_settings.py), and the
dashboard buttons for autotune and steering encoder zeroing.

    drivetrain = Drivetrain()
    drivetrain.driveWithController(controller)
"""

import math

import commands2
import wpilib
import wpimath
from wpimath.controller import PIDController

from ..tuning.autotune import Autotuner
from ..vision.vision import VisionSubsystem
from .constants import DriveConstants, OIConstants
from .drivesubsystem import DriveSubsystem


class Drivetrain(DriveSubsystem):
    def __init__(self) -> None:
        super().__init__()

        # Wheel positions on the robot, for WPILib kinematics and odometry
        self.kinematics = DriveConstants.kDriveKinematics

        # Cameras correct the robot's position when available (off until
        # VISION_ENABLED = True in robot_settings.py)
        self.vision = VisionSubsystem(self)

        # Dashboard: robot position on a field map
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Robot Position", self.field)

        # Dashboard buttons for autotune (Test mode only) and encoder zeroing
        self.autotuner = Autotuner(self)
        sd = wpilib.SmartDashboard
        sd.putData("Autotune/1 Steering (robot stays put)", self.autotuner.steeringCommand())
        sd.putData("Autotune/2 Drive (needs clear floor ahead)", self.autotuner.driveCommand())
        sd.putData("Autotune/All", self.autotuner.fullCommand())
        sd.putData("Autotune/Reset to defaults", self.autotuner.resetCommand())
        sd.putData(
            "Calibrate/Zero steering encoders (hex keys in, disabled)",
            commands2.InstantCommand(self._zeroSteering).ignoringDisable(True),
        )

    def periodic(self) -> None:
        super().periodic()
        self.field.setRobotPose(self.getPose())

    def _zeroSteering(self) -> None:
        msg = self.zeroSteeringEncoders()
        wpilib.SmartDashboard.putString("Calibrate/Status", msg)
        print(f"[calibrate] {msg}")

    # ---- easy commands for robot.py --------------------------------------------

    def driveWithController(self, controller, fieldRelative: bool = True) -> None:
        """Drive with an Xbox controller whenever nothing else is using the
        drivetrain: left stick moves, right stick turns.

        :param controller: a CommandXboxController (or XboxController)
        :param fieldRelative: True = pushing forward always drives away from
                              the driver, whichever way the robot faces
        """
        deadband = OIConstants.kDriveDeadband

        def driveFromSticks() -> None:
            # Sticks read negative when pushed forward or left, hence the minus
            self.drive(
                -wpimath.applyDeadband(controller.getLeftY(), deadband),
                -wpimath.applyDeadband(controller.getLeftX(), deadband),
                -wpimath.applyDeadband(controller.getRightX(), deadband),
                fieldRelative,
            )

        self.setDefaultCommand(commands2.RunCommand(driveFromSticks, self))

    def lockWheels(self) -> commands2.Command:
        """Point the wheels in an X so the robot is hard to push (while held)."""
        return commands2.RunCommand(self.setX, self)

    def resetHeading(self) -> commands2.Command:
        """Make the direction the robot faces now count as 'forward'."""
        return commands2.InstantCommand(self.zeroHeading, self)

    def stop(self) -> commands2.Command:
        """Stop all wheels."""
        return commands2.InstantCommand(lambda: self.drive(0, 0, 0, False), self)

    # ---- calibration ------------------------------------------------------------------

    def zeroSteeringCommand(self) -> commands2.Command:
        """Save the current wheel angles as straight ahead (hex keys in, robot
        disabled). The same as the dashboard's Calibrate button; works while
        disabled, so it can go on a controller button."""
        return commands2.InstantCommand(self._zeroSteering).ignoringDisable(True)

    def gyroCalibrating(self) -> bool:
        """True while the navX is still calibrating after power-on (keep the
        robot still until this is False)."""
        try:
            return bool(self.gyro.isCalibrating())
        except Exception:
            return False

    def saveGains(self, gains) -> str:
        """Use these gains now and save them on the roboRIO, so they are
        loaded at every startup. Returns where they were saved."""
        from ..tuning import gains as savedGains

        path = savedGains.save(gains)
        self.applyGains(gains)
        return path

    def module(self, name: str):
        """One swerve module by name: "FL", "FR", "RL" or "RR"
        (front left, front right, rear left, rear right)."""
        for n, m in self.modules:
            if n == name.upper():
                return m
        raise ValueError(f'module name must be "FL", "FR", "RL" or "RR", not {name!r}')

    def checkup(self) -> list:
        """Checks everything the drivetrain can check without moving:
        settings, every SPARK MAX, every steering encoder, the navX, the
        battery, zeros and gains. Returns (ok, message) pairs."""
        import robot_settings as RS

        results = []

        def add(ok, text):
            results.append((bool(ok), text))

        # settings: duplicate CAN IDs or DIO ports are a common mistake
        mods = {"FRONT_LEFT": RS.FRONT_LEFT, "FRONT_RIGHT": RS.FRONT_RIGHT,
                "BACK_LEFT": RS.BACK_LEFT, "BACK_RIGHT": RS.BACK_RIGHT}
        cans = [(f"{n} drive_can", m.drive_can) for n, m in mods.items()]
        cans += [(f"{n} turn_can", m.turn_can) for n, m in mods.items()]
        seen = {}
        for label, value in cans:
            seen.setdefault(value, []).append(label)
        dup = [f"{v} ({', '.join(ls)})" for v, ls in seen.items() if len(ls) > 1]
        add(not dup, "CAN IDs all different" if not dup else f"same CAN ID used twice: {'; '.join(dup)}")
        if self.frontLeft.turningEncoder.location == "rio":
            dios = {}
            for n, m in mods.items():
                dios.setdefault(m.encoder_dio, []).append(n)
            dup = [f"{v} ({', '.join(ls)})" for v, ls in dios.items() if len(ls) > 1]
            add(not dup, "encoder DIO ports all different" if not dup else f"same DIO port used twice: {'; '.join(dup)}")

        # every SPARK MAX: answering on CAN? any faults?
        sim = wpilib.RobotBase.isSimulation()
        for name, m in self.modules:
            for role, spark in (("drive", m.drivingSpark), ("steer", m.turningSpark)):
                try:
                    can = spark.getDeviceId()
                    volts = spark.getBusVoltage()
                    firmware = spark.getFirmwareString()
                    answering = sim or volts > 1.0
                    if not answering:
                        add(False, f"{name} {role} SPARK MAX (CAN {can}) not answering: check its CAN ID "
                                   "in REV Hardware Client and the CAN wiring")
                        continue
                    fault = spark.hasActiveFault()
                    add(not fault, f"{name} {role} SPARK MAX (CAN {can}) answering, firmware {firmware}"
                                   + (", HAS AN ACTIVE FAULT (see REV Hardware Client)" if fault else ""))
                except Exception as e:
                    add(False, f"{name} {role} SPARK MAX: could not read it ({e})")

        # steering encoders
        for name, m in self.modules:
            enc = m.turningEncoder
            if enc.location == "rio":
                ok = enc.isConnected()
                add(ok, f"{name} steering encoder on DIO {enc.dioChannel} "
                        + ("sending a signal" if ok else "NO SIGNAL: check its 3 wires (signal, 5 V, ground) and the DIO number"))
                if getattr(enc, "stuck", False):
                    add(False, f"{name} steering encoder reads the same value while the wheel turns (magnet or wiring)")

        # gyro
        try:
            ok = bool(self.gyro.isConnected())
        except Exception:
            ok = False
        add(ok, "navX connected" if ok else f'navX NOT connected (GYRO = "{RS.GYRO}" in robot_settings.py; check the port)')
        if ok and self.gyroCalibrating():
            add(True, "navX still calibrating: keep the robot still")

        # battery
        v = wpilib.RobotController.getBatteryVoltage()
        add(v >= 12.0 or sim, f"battery {v:.1f} V" + ("" if v >= 12.0 or sim else " (charge it: under 12 V the tests are less accurate)"))

        # zeros and gains
        zs = getattr(self, "steeringZeroSource", "")
        add("NOT ZEROED" not in zs, f"steering zeros: {zs}")
        add(True, f"gains: {self.gains.summary()}")
        return results

    def wheelDistances(self) -> list:
        """How far each wheel has rolled, in meters (FL, FR, RL, RR), as the
        drive encoders measure it."""
        return [m.drivingEncoder.getPosition() for _, m in self.modules]

    # ---- simple moves for autos -------------------------------------------------
    # Speeds are fractions of full speed: 1.0 = full, 0.5 = half, negative =
    # the other way. "forward", "left" and "turn" are from the robot's point
    # of view (turn: + = counterclockwise, seen from above).

    def driveFor(
        self, seconds: float, forward: float = 0.0, left: float = 0.0, turn: float = 0.0
    ) -> commands2.Command:
        """Drive at a fixed speed for some time, then stop.

        Example: drivetrain.driveFor(2.0, forward=0.5)   # half speed, 2 seconds
        """
        return commands2.FunctionalCommand(
            lambda: None,
            lambda: self.drive(forward, left, turn, False),
            lambda interrupted: self.drive(0, 0, 0, False),
            lambda: False,
            self,
        ).withTimeout(seconds)

    def driveDistance(self, meters: float, speed: float = 0.3) -> commands2.Command:
        """Drive straight forward (negative meters = backward) until the robot
        has gone that far, then stop.

        Example: drivetrain.driveDistance(2.0)
        """
        start = {}
        direction = 1.0 if meters >= 0 else -1.0

        def begin() -> None:
            start["pose"] = self.getPose()

        def done() -> bool:
            traveled = self.getPose().translation().distance(start["pose"].translation())
            return traveled >= abs(meters)

        return commands2.FunctionalCommand(
            begin,
            lambda: self.drive(direction * abs(speed), 0, 0, False),
            lambda interrupted: self.drive(0, 0, 0, False),
            done,
            self,
        )

    def turnTo(self, headingDeg: float, maxTurnSpeed: float = 0.5) -> commands2.Command:
        """Spin in place until the robot faces a direction, then stop.
        0 = the direction it faced at start (or at the last resetHeading),
        90 = a quarter turn counterclockwise (left), -90 = right, 180 = around.

        Example: drivetrain.turnTo(90)
        """
        pid = PIDController(0.015, 0.0, 0.0)  # turn speed per degree of error
        pid.enableContinuousInput(-180.0, 180.0)  # 179 and -179 are 2 apart
        pid.setTolerance(2.0)  # close enough: within 2 degrees

        def arrived() -> bool:
            # within 2 degrees AND nearly stopped, so it doesn't coast past
            return pid.atSetpoint() and abs(self.getTurnRate()) < 10.0

        def step() -> None:
            out = pid.calculate(self.getHeading(), headingDeg)
            # at least a little turn speed (friction), at most maxTurnSpeed
            if abs(out) < 0.04 and not pid.atSetpoint():
                out = math.copysign(0.04, out)
            self.drive(0, 0, max(-maxTurnSpeed, min(maxTurnSpeed, out)), False)

        return commands2.FunctionalCommand(
            pid.reset,
            step,
            lambda interrupted: self.drive(0, 0, 0, False),
            arrived,
            self,
        )
