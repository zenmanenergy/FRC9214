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
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

from ..tuning.autotune import Autotuner
from ..vision.vision import VisionSubsystem
from .constants import DriveConstants, OIConstants
from .drivesubsystem import DriveSubsystem
from .positioncontrol import PositionController



def firmwareYear(firmware: str):
    """The first number of a SPARK firmware string ("v26.1.0" -> 26), or None."""
    import re

    found = re.search(r"(\d+)\.", str(firmware))
    return int(found.group(1)) if found else None


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
        sd.putData("Autotune/2 Drive (start at one end of the carpet)", self.autotuner.driveCommand())
        sd.putData("Autotune/3 Position (start at one end of the carpet)", self.autotuner.positionCommand())
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
                    refused = m.configProblems.get(role)
                    if refused:
                        add(False, f"{name} {role} SPARK MAX (CAN {can}) REFUSED ITS SETTINGS ({refused}). "
                                   f"Firmware {firmware}: this code needs 2026 firmware (26.x), put on with REV Hardware Client")
                        continue
                    year = None if sim else firmwareYear(firmware)
                    if year is not None and year != 26:
                        which = "too old: update it" if year < 26 else "too new (2027 or later): put it back"
                        add(False, f"{name} {role} SPARK MAX (CAN {can}) firmware {firmware} is {which} "
                                   "to 2026 firmware (26.x) with REV Hardware Client")
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
        """Drive straight forward (negative meters = backward) exactly that far,
        and stop on the spot, still facing the same way.

        The target is a point on the field: where the robot started, plus
        that many meters straight ahead. The tuned position PID (posP/I/D,
        from setup step 11) compares where the robot is with that point
        every loop and asks for the speed that closes the gap: fast when
        far, slowing down as it gets close, and a push back if it rolls
        past. The tuned heading PID keeps it facing the same way. The speed
        PID inside each SPARK MAX makes the wheels go that speed.

        Example: drivetrain.driveDistance(2.0)
        """

        def target(start):
            ahead = start.rotation()
            dx, dy = meters * ahead.cos(), meters * ahead.sin()
            return Pose2d(start.X() + dx, start.Y() + dy, ahead)

        maxMps = abs(speed) * DriveConstants.kMaxSpeedMetersPerSecond
        seconds = abs(meters) / max(maxMps, 0.05) + DriveConstants.kDistanceTimeoutS
        return self.moveTo(target, maxMps, DriveConstants.kMaxAngularSpeed * 0.5, seconds)

    def turnTo(self, headingDeg: float, maxTurnSpeed: float = 0.5) -> commands2.Command:
        """Spin in place until the robot faces a direction, then stop.
        0 = the direction it faced at start (or at the last resetHeading),
        90 = a quarter turn counterclockwise (left), -90 = right, 180 = around.

        Uses the tuned heading PID (headP/I/D, from setup step 11), and the
        position PID keeps the robot on the same spot while it turns.

        Example: drivetrain.turnTo(90)
        """

        def target(start):
            turn = headingDeg - self.getHeading()  # the gyro says how far to turn
            return Pose2d(start.translation(), start.rotation() + Rotation2d.fromDegrees(turn))

        maxRad = abs(maxTurnSpeed) * DriveConstants.kMaxAngularSpeed
        seconds = math.pi / max(maxRad, 0.1) + DriveConstants.kDistanceTimeoutS
        return self.moveTo(target, DriveConstants.kMaxSpeedMetersPerSecond * 0.3, maxRad, seconds)

    def moveTo(self, makeTarget, maxMps: float, maxRadPerS: float, seconds: float) -> commands2.Command:
        """A command: drive to a pose with the tuned position and heading PID.
        makeTarget(startPose) returns the target, worked out when the command
        starts. Done when it has stayed within 1 cm and 1 degree, nearly
        stopped, for 0.1 s (so it isn't "done" while rolling through the
        target), or after the given seconds (it prints how far off it was)."""
        state = {}

        def begin() -> None:
            state["target"] = makeTarget(self.getPose())
            state["inside"] = 0  # loops in a row inside the tolerance
            state["pid"] = PositionController(
                self.gains, maxMps, maxRadPerS, DriveConstants.kDistanceAccelMps2
            )

        def run() -> None:
            pose = self.getPose()
            vx, vy, omega = state["pid"].calculate(pose, state["target"])
            self.driveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, pose.rotation()))

        def errors():
            pose, goal = self.getPose(), state["target"]
            meters = pose.translation().distance(goal.translation())
            degrees = abs((pose.rotation() - goal.rotation()).degrees())
            return meters, degrees

        def done() -> bool:
            meters, degrees = errors()
            stopped = max(abs(m.wheelSpeed()) for _, m in self.modules) < 0.05
            inside = (
                meters <= DriveConstants.kDistanceToleranceM
                and degrees <= DriveConstants.kDistanceHeadingToleranceDeg
                and stopped
                and abs(self.getTurnRate()) < 10.0
            )
            state["inside"] = state["inside"] + 1 if inside else 0
            return state["inside"] >= 5  # 5 loops = 0.1 s

        def end(interrupted: bool) -> None:
            self.drive(0, 0, 0, False)
            if interrupted and "target" in state:
                meters, degrees = errors()
                if meters > 0.03 or degrees > 3.0:
                    print(f"[drive] move stopped {meters * 100:.0f} cm / {degrees:.0f} deg from the target")

        return commands2.FunctionalCommand(begin, run, end, done, self).withTimeout(seconds)

