#
# Simulation model for the EasySwerve drivetrain. (Loaded through physics.py in
# the project folder, which RobotPy finds automatically.)
#
# RobotPy loads it when you run
#     python -m robotpy sim      (GUI)
#     python -m robotpy test     (automated tests)
# It is never deployed to or run on the real robot.
#
# What it does every 20 ms, in 1 ms sub-steps:
#   1. Works out the voltage each SPARK MAX would apply: either the open-loop
#      voltage autotune asked for, or the SPARK's own closed loop (P and
#      feedforward, using the gains the code currently has applied).
#   2. Moves each simulated motor with a DC motor model
#      (volts = kS*sign(v) + kV*v + kA*a), using the "true" constants below.
#   3. Writes the results into the simulated encoders, so the robot code reads
#      them exactly as it reads real ones (drive speed with a measurement lag,
#      like the real SPARK).
#   4. Moves the robot on the sim Field2d ("ground truth") and turns the navX.
#   5. Prints a one-line status, so you can copy the terminal output.
#
# Autotune run in the simulator should recover the SIM_* constants below,
# which is how the tuning code itself is checked.
#
# Not modeled: motor inversion, encoder zeroing, wheel slip, battery sag,
# current limits. Those need the real robot.
#

import collections
import math
import typing

import wpilib
from wpilib.simulation import DutyCycleEncoderSim, SimDeviceSim
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

from rev import SparkAbsoluteEncoderSim, SparkRelativeEncoderSim

from ..drivetrain.constants import DriveConstants, ModuleConstants

# ---- settings you may want to change ----------------------------------------

# Print a status line this often while enabled (seconds). 0 turns it off.
PRINT_PERIOD_S = 0.5

# "True" motor constants of the simulated robot (volts, rad or m, s).
# Roughly an EasySwerve robot; change them to see how autotune responds.
SIM_STEER_KS = 0.15  # V
SIM_STEER_KV = 0.25  # V per rad/s
SIM_STEER_KA = 0.006  # V per rad/s^2
SIM_DRIVE_KS = 0.12  # V
SIM_DRIVE_KV = 2.45  # V per m/s (6.3 : 1 gearing predicts about 2.50)
SIM_DRIVE_KA = 0.35  # V per m/s^2 (depends on robot mass)

# How far the simulated SPARK's drive speed reading lags the real speed (s):
# half the averaging window set in constants.py, plus about 10 ms for the
# reading to reach the roboRIO.
SIM_DRIVE_SPEED_LAG_S = (
    ModuleConstants.kDriveVelocityMeasurementPeriodMs
    * ModuleConstants.kDriveVelocityAverageDepth
    / 2.0
    / 1000.0
    + 0.010
)

SUBSTEPS = 20  # physics sub-steps per 20 ms loop (1 ms, like the SPARK loop)
# The SPARKs run with voltage compensation, so a duty-cycle command means this
# many volts whatever the battery voltage.
NOMINAL_VOLTS = ModuleConstants.kNominalVoltage

# Starting steering angle of every module, radians in the encoder's own frame.
# 0 = the hex-key zero position. Set to e.g. 1.0 to check that the code
# straightens the wheels out on enable.
INITIAL_STEER_ANGLE_RAD = 0.0

# Wheel slip: fraction of wheel travel that doesn't move the robot (0.05 = the
# robot goes 5 % less far than the encoders say). Real robots slip; the
# simulated one doesn't unless you set this. Use it to watch odometry drift and
# a camera (vision add-on) pull the position back.
SIM_WHEEL_SLIP = 0.0

TAU = 2 * math.pi


def _wrap0to2pi(a: float) -> float:
    return a % TAU


def _shortestDiff(target: float, current: float) -> float:
    """Signed smallest angle from current to target, in (-pi, pi]."""
    d = (target - current) % TAU
    return d - TAU if d > math.pi else d


class _Motor:
    """First-order DC motor with static friction: V = kS*sign(v) + kV*v + kA*a."""

    def __init__(self, kS: float, kV: float, kA: float) -> None:
        self.kS, self.kV, self.kA = kS, kV, kA
        self.v = 0.0
        self.x = 0.0

    def step(self, volts: float, h: float) -> None:
        if abs(self.v) < 1e-6 and abs(volts) <= self.kS:
            self.v = 0.0  # stuck: static friction holds it
        else:
            direction = math.copysign(1.0, self.v if abs(self.v) >= 1e-6 else volts)
            accel = (volts - self.kS * direction - self.kV * self.v) / self.kA
            newV = self.v + accel * h
            # friction can stop the motor but never reverse it on its own
            if abs(volts) <= self.kS and newV * self.v < 0:
                newV = 0.0
            self.v = newV
        self.x += self.v * h


def _clampVolts(v: float) -> float:
    return max(-NOMINAL_VOLTS, min(NOMINAL_VOLTS, v))


class _SimModule:
    def __init__(self, name: str, module) -> None:
        self.name = name
        self.module = module
        # Steering angle goes wherever the robot code reads it: the turning
        # SPARK's absolute encoder (SPARK wiring) or, with roboRIO wiring, the
        # turning NEO's own sensor plus the DIO absolute encoder.
        self.rioWiring = module.turningEncoder.location == "rio"
        if self.rioWiring:
            self.absSim = SparkRelativeEncoderSim(module.turningSpark)
            self.dioSim = DutyCycleEncoderSim(module.turningEncoder.absolute)
            self.dioSim.setConnected(True)
        else:
            self.absSim = SparkAbsoluteEncoderSim(module.turningSpark)
            self.dioSim = None
        self.relSim = SparkRelativeEncoderSim(module.drivingSpark)

        self.steer = _Motor(SIM_STEER_KS, SIM_STEER_KV, SIM_STEER_KA)
        self.steer.x = INITIAL_STEER_ANGLE_RAD
        self.drive = _Motor(SIM_DRIVE_KS, SIM_DRIVE_KV, SIM_DRIVE_KA)
        self.speedHistory = collections.deque()  # (time, speed) for the lagged reading
        self.measuredSpeed = 0.0

        # Work out whether the encoder simulators expect raw sensor units or
        # converted units (rad, m), by writing 1.0 and reading it back through
        # the real encoder objects the robot code uses.
        self.absScale = self._detectScale(self.absSim, module.turningEncoder)
        self.relScale = self._detectScale(self.relSim, module.drivingEncoder)

        self._write()

    @staticmethod
    def _detectScale(sim, encoder) -> float:
        sim.setPosition(1.0)
        read = encoder.getPosition()
        sim.setPosition(0.0)
        if abs(read) < 1e-9:
            return 1.0
        return read

    @property
    def angle(self) -> float:
        return _wrap0to2pi(self.steer.x)

    @property
    def speed(self) -> float:
        return self.drive.v

    def _write(self) -> None:
        self.absSim.setPosition(self.angle / self.absScale)
        self.absSim.setVelocity(self.steer.v / self.absScale)
        if self.dioSim is not None:
            # what the DIO encoder reads: true angle plus its mounting offset
            self.dioSim.set((self.angle + self.module.turningEncoder.offset) % (2 * math.pi))
        self.relSim.setPosition(self.drive.x / self.relScale)
        self.relSim.setVelocity(self.measuredSpeed / self.relScale)

    def _steerVolts(self, enabled: bool) -> float:
        m = self.module
        if not enabled:
            return 0.0
        if m.turningVoltage is not None:
            return _clampVolts(m.turningVoltage)
        # SPARK position loop: P (duty cycle, wrapping enabled) plus the
        # friction push the robot code sent with the setpoint (volts)
        target = _wrap0to2pi(m.turningClosedLoopController.getSetpoint())
        err = _shortestDiff(target, self.angle)
        return _clampVolts(NOMINAL_VOLTS * m.gains.steerP * err + m.turningFeedforward)

    def _driveVolts(self, enabled: bool) -> float:
        m = self.module
        if not enabled:
            return 0.0
        if m.drivingVoltage is not None:
            return _clampVolts(m.drivingVoltage)
        # SPARK velocity loop: feedforward kS, kV plus P on the (lagged) reading
        target = m.drivingClosedLoopController.getSetpoint()
        g = m.gains
        ff = g.driveKV * target + (math.copysign(g.driveKS, target) if target != 0 else 0.0)
        fb = NOMINAL_VOLTS * g.driveP * (target - self.measuredSpeed)
        return _clampVolts(ff + fb)

    def update(self, now: float, dt: float, enabled: bool) -> None:
        h = dt / SUBSTEPS
        for i in range(SUBSTEPS):
            self.steer.step(self._steerVolts(enabled), h)
            self.drive.step(self._driveVolts(enabled), h)
            t = now - dt + (i + 1) * h
            self.speedHistory.append((t, self.drive.v))
            while len(self.speedHistory) > 1 and self.speedHistory[1][0] <= t - SIM_DRIVE_SPEED_LAG_S:
                self.speedHistory.popleft()
            self.measuredSpeed = self.speedHistory[0][1]
        self._write()

    def robotRelativeState(self) -> SwerveModuleState:
        # Same transform the module code uses: encoder angle minus chassis offset
        return SwerveModuleState(
            self.speed, Rotation2d(self.angle - self.module.chassisAngularOffset)
        )


class _NavxSim:
    """Feeds the simulated heading into the navX.

    The navX simulation device's name differs between library versions, so
    this finds it by name. If it can't, it falls back to feeding the heading
    straight into the drive subsystem and says so in the terminal.
    """

    def __init__(self, drive) -> None:
        self.drive = drive
        self.headingRad = 0.0
        self.rateRadS = 0.0
        self.yaw = None
        self.angle = None
        self.rate = None

        names = [
            n for n in SimDeviceSim.enumerateDevices("")
            if "navx" in n.lower()
        ]
        for name in names:
            dev = SimDeviceSim(name)
            yaw = dev.getDouble("Yaw")
            if yaw:
                self.yaw = yaw
                angle = dev.getDouble("Angle")
                self.angle = angle if angle else None
                rate = dev.getDouble("Rate")
                self.rate = rate if rate else None
                print(f"[physics] navX simulated through sim device '{name}'")
                break

        if self.yaw is None:
            print(
                "[physics] navX sim device not found "
                f"(saw: {names or 'none'}); feeding heading directly into DriveSubsystem"
            )
            drive._gyroRotation = lambda: Rotation2d(self.headingRad)
            drive.getTurnRate = lambda: math.degrees(self.rateRadS)

    def update(self, headingRad: float, rateRadS: float) -> None:
        self.headingRad = headingRad
        self.rateRadS = rateRadS
        if self.yaw is None:
            return
        # navX is clockwise-positive; WPILib (and our model) counterclockwise-positive
        cwDeg = -math.degrees(headingRad)
        self.yaw.set(((cwDeg + 180) % 360) - 180)
        if self.angle is not None:
            self.angle.set(cwDeg)
        if self.rate is not None:
            self.rate.set(-math.degrees(rateRadS))


def findDrivetrain(robot):
    """The robot's drivetrain, whatever robot.py named it."""
    from ..drivetrain.drivesubsystem import DriveSubsystem

    container = getattr(robot, "container", None)
    for holder in (container, robot):
        if holder is None:
            continue
        for value in vars(holder).values():
            if isinstance(value, DriveSubsystem):
                return value
    raise RuntimeError(
        "physics.py: no Drivetrain found. robot.py must create one in robotInit, "
        "for example self.drivetrain = Drivetrain()."
    )


class PhysicsEngine:
    def __init__(self, physics_controller, robot) -> None:
        self.pc = physics_controller
        self.drive = findDrivetrain(robot)
        d = self.drive

        self.modules = [
            _SimModule("FL", d.frontLeft),
            _SimModule("FR", d.frontRight),
            _SimModule("RL", d.rearLeft),
            _SimModule("RR", d.rearRight),
        ]
        self.navx = _NavxSim(d)
        self.headingRad = 0.0  # continuous, counterclockwise-positive
        self.lastPrint = -1e9
        self.lastOdo = d.getPose()
        self.lastResetCount = getattr(d, "poseResetCount", 0)

        # Optional: simulated cameras (vision add-on). It sees the simulated
        # AprilTags from the robot's true position.
        self.vision = getattr(self.drive, "vision", None) or getattr(
            getattr(robot, "container", None), "vision", None
        )

        print(
            "[physics] EasySwerve sim ready: encoder scale "
            f"abs={self.modules[0].absScale:.4f} rel={self.modules[0].relScale:.4f} "
            "(1.0 = sim takes radians/meters directly)"
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        enabled = wpilib.DriverStation.isEnabled()

        # If the robot code reset its position (start of an auto, or a manual
        # resetOdometry call), that means "the robot is placed here". Move the
        # ground-truth robot there too, so the position error stays meaningful.
        # (Vision corrections also move the estimate, but must not move the
        # true robot, so this watches for resets only.)
        odo = self.drive.getPose()
        resets = getattr(self.drive, "poseResetCount", 0)
        if resets != self.lastResetCount:
            self.lastResetCount = resets
            self.pc.field.setRobotPose(odo)
            print(
                f"[physics] odometry reset detected; sim robot placed at "
                f"x={odo.X():.2f} y={odo.Y():.2f} h={odo.rotation().degrees():.1f}"
            )

        for m in self.modules:
            m.update(now, tm_diff, enabled)

        speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(
            tuple(m.robotRelativeState() for m in self.modules)
        )

        # Move the ground-truth robot on the sim field
        if SIM_WHEEL_SLIP:
            k = 1.0 - SIM_WHEEL_SLIP
            speeds = type(speeds)(speeds.vx * k, speeds.vy * k, speeds.omega * k)
        truth = self.pc.drive(speeds, tm_diff)
        if self.vision is not None and hasattr(self.vision, "simulationUpdate"):
            self.vision.simulationUpdate(truth)

        self.headingRad += speeds.omega * tm_diff
        self.navx.update(self.headingRad, speeds.omega)

        self.lastOdo = self.drive.getPose()

        if PRINT_PERIOD_S > 0 and enabled and now - self.lastPrint >= PRINT_PERIOD_S:
            self.lastPrint = now
            self._printStatus(now, truth)

    def _printStatus(self, now: float, truth) -> None:
        odo = self.drive.getPose()
        err = odo.translation().distance(truth.translation())
        mode = "auto" if wpilib.DriverStation.isAutonomous() else "teleop"
        mods = " ".join(
            f"{m.name} {math.degrees(m.robotRelativeState().angle.radians()):6.1f}deg "
            f"{m.speed:5.2f}m/s"
            for m in self.modules
        )
        print(
            f"t={now:7.2f} {mode:6} "
            f"truth x={truth.X():6.2f} y={truth.Y():6.2f} h={truth.rotation().degrees():7.1f} | "
            f"odo x={odo.X():6.2f} y={odo.Y():6.2f} h={odo.rotation().degrees():7.1f} "
            f"err={err:5.3f}m | {mods}"
        )
