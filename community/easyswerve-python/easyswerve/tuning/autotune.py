"""
Autotune for the EasySwerve drivetrain.

Runs from the dashboard in Test mode. Three entry points:

  steeringCommand()  robot stays in place; finds steering P       (about 30 s)
  driveCommand()     robot drives up to kDriveMaxDistanceM forward and back;
                     finds drive kS, kV and P                      (about 25 s)
  fullCommand()      steering, then drive

How it works:

  Steering
    1. Open-loop voltage tests (slow ramp + sudden step, both directions) on
       all 4 steering motors at once, recording the absolute encoder angle.
    2. Fit kS, kV, kA per module (tuning/fit.py) and take the median.
    3. Starting P from the model for the target damping ratio.
    4. Check-and-adjust: real 60 degree steps on all 4 wheels. Too much
       overshoot lowers P, a sluggish response raises it, until the response
       is crisp without ringing.

  Drive
    1. Wheels held straight ahead. Open-loop voltage tests on the 4 drive
       motors, forward then backward so the robot ends near where it started.
    2. Fit kS, kV, kA per module; median.
    3. Measure how far the SPARK's speed measurement lags real speed.
    4. Feedforward kS, kV straight from the fit; P from the SIMC rule using
       that lag (fast without oscillating).

Results are saved (tuning/gains.py), applied to the SPARKs immediately, and
loaded automatically at every startup. A report is printed to the console
and summarized on the dashboard under Autotune/.

Safety: Test mode only (constants.AutotuneConstants.kRequireTestMode),
disabling the robot stops it at once, and the drive test stops at
kDriveMaxDistanceM.
"""

import dataclasses
import math
import typing

import commands2
import wpilib
from wpimath.geometry import Rotation2d

from ..drivetrain.constants import AutotuneConstants as AC
from ..drivetrain.constants import ModuleConstants
from . import fit
from . import gains as G

STEER = "steer"
DRIVE = "drive"


def _now() -> float:
    return wpilib.Timer.getFPGATimestamp()


def _wrapPi(a: float) -> float:
    return (a + math.pi) % math.tau - math.pi


class _Unwrapper:
    """Turns a 0..2pi encoder angle into a continuous angle."""

    def __init__(self, first: float) -> None:
        self.last = first
        self.total = first

    def __call__(self, raw: float) -> float:
        self.total += _wrapPi(raw - self.last)
        self.last = raw
        return self.total


class _VoltageRun(commands2.Command):
    """Applies a voltage profile to one axis of all four modules and records
    (time, volts, position) every loop."""

    def __init__(
        self,
        tuner: "Autotuner",
        axis: str,
        label: str,
        profile: typing.Callable[[float], float],
        maxSeconds: float,
        distanceLimitM: typing.Optional[float] = None,
    ) -> None:
        super().__init__()
        self.tuner = tuner
        self.axis = axis
        self.label = label
        self.profile = profile
        self.maxSeconds = maxSeconds
        self.distanceLimitM = distanceLimitM
        self.addRequirements(tuner.drive)

    def initialize(self) -> None:
        self.t0 = _now()
        self.elapsed = 0.0
        mods = self.tuner.drive.modules
        self.samples = {name: [] for name, _ in mods}
        self.measured = {name: [] for name, _ in mods}
        self.unwrap = {name: _Unwrapper(m.turningEncoder.getPosition()) for name, m in mods}
        self.start = {name: m.drivingEncoder.getPosition() for name, m in mods}
        self.tuner.data.setdefault(self.axis, {})[self.label] = (self.samples, self.measured)
        self.tuner.status(f"{self.axis} test: {self.label}")

    def execute(self) -> None:
        now = _now()
        self.elapsed = now - self.t0
        volts = max(-12.0, min(12.0, self.profile(self.elapsed)))
        for name, m in self.tuner.drive.modules:
            if self.axis == STEER:
                pos = self.unwrap[name](m.turningEncoder.getPosition())
                meas = 0.0
                m.setDrivingVoltage(0.0)
                m.setTurningVoltage(volts)
            else:
                pos = m.drivingEncoder.getPosition()
                meas = m.drivingEncoder.getVelocity()
                m.setDrivingVoltage(volts)
            self.samples[name].append((now, volts, pos))
            self.measured[name].append(meas)

    def isFinished(self) -> bool:
        if self.elapsed >= self.maxSeconds:
            return True
        if self.axis == DRIVE and self.distanceLimitM is not None:
            mods = self.tuner.drive.modules
            traveled = sum(
                abs(m.drivingEncoder.getPosition() - self.start[name]) for name, m in mods
            ) / len(mods)
            if traveled >= self.distanceLimitM:
                return True
        return False

    def end(self, interrupted: bool) -> None:
        for _, m in self.tuner.drive.modules:
            if self.axis == STEER:
                m.setTurningVoltage(0.0)
            else:
                m.setDrivingVoltage(0.0)
        if interrupted:
            self.tuner.fail("stopped (robot disabled or command interrupted)")


class _AlignWheels(commands2.Command):
    """Point every wheel straight ahead (no flip optimization) and wait until
    they get there."""

    def __init__(self, tuner: "Autotuner", timeout: float = 2.0) -> None:
        super().__init__()
        self.tuner = tuner
        self.timeout = timeout
        self.addRequirements(tuner.drive)

    def initialize(self) -> None:
        self.t0 = _now()
        self.tuner.status("aligning wheels straight ahead")
        for _, m in self.tuner.drive.modules:
            m.setDrivingVoltage(0.0)
            m.holdAngle(Rotation2d())

    def _worstErrorDeg(self) -> float:
        return max(
            abs(math.degrees(_wrapPi(m.getState().angle.radians())))
            for _, m in self.tuner.drive.modules
        )

    def isFinished(self) -> bool:
        t = _now() - self.t0
        return t >= 0.5 and (self._worstErrorDeg() < 3.0 or t >= self.timeout)

    def end(self, interrupted: bool) -> None:
        if interrupted:
            self.tuner.fail("stopped while aligning wheels")
        elif self._worstErrorDeg() >= 3.0:
            self.tuner.fail(
                f"wheels did not line up straight (worst {self._worstErrorDeg():.1f} deg); "
                "check encoder zeroing and steering, or run steering autotune first"
            )


class _SteerCheckAndAdjust(commands2.Command):
    """Repeated real steering steps; adjusts P from measured overshoot."""

    SETTLE_S = 0.5
    STEP_S = 0.8

    def __init__(self, tuner: "Autotuner") -> None:
        super().__init__()
        self.tuner = tuner
        self.addRequirements(tuner.drive)

    def initialize(self) -> None:
        self.iteration = 0
        self.good: typing.Optional[float] = None
        self.done = False
        self.p = self.tuner.steerP
        self.tuner.applySteerP(self.p)
        self.target = 0.0
        self.stepSize = math.radians(AC.kSteerVerifyStepDegrees)
        self._hold(self.target)
        self.phase = "settle"
        self.phaseStart = _now()
        self.tuner.status("steering check-and-adjust running")

    def _hold(self, angle: float) -> None:
        for _, m in self.tuner.drive.modules:
            m.setDrivingVoltage(0.0)
            m.holdAngle(Rotation2d(angle))

    def execute(self) -> None:
        now = _now()
        t = now - self.phaseStart
        if self.phase == "settle":
            if t >= self.SETTLE_S:
                self.stepFrom = self.target
                self.target = self.stepSize if self.target == 0.0 else 0.0
                self.times: typing.List[float] = []
                self.angles = {name: [] for name, _ in self.tuner.drive.modules}
                self._hold(self.target)
                self.phase = "step"
                self.phaseStart = now
            return

        # phase == "step": record
        self.times.append(now)
        for name, m in self.tuner.drive.modules:
            a = m.getState().angle.radians()
            self.angles[name].append(self.target + _wrapPi(a - self.target))
        if t >= self.STEP_S:
            self._evaluate()
            self.phase = "settle"
            self.phaseStart = now

    def _evaluate(self) -> None:
        worstOver, worstSettle, worstName = 0.0, 0.0, ""
        for name, angles in self.angles.items():
            moved = abs(angles[-1] - self.stepFrom)
            if moved < 0.5 * self.stepSize:
                self.tuner.fail(
                    f"module {name} steered only {math.degrees(moved):.0f} of "
                    f"{AC.kSteerVerifyStepDegrees:.0f} deg; check steering motor and encoder"
                )
                self.done = True
                return
            over, settle = fit.stepResponse(self.times, angles, self.stepFrom, self.target)
            if over > worstOver:
                worstOver, worstName = over, name
            worstSettle = max(worstSettle, settle)

        self.tuner.report(
            f"  step {self.iteration + 1}: P={self.p:.3f}  worst overshoot "
            f"{100 * worstOver:.1f}% {worstName}  settle {worstSettle:.2f} s"
        )
        self.tuner.steerHistory.append((self.p, worstOver, worstSettle))

        nextP = None
        if worstOver > AC.kSteerOvershootMax:
            if self.good is not None:
                self.done = True  # previous P was fine; go back to it
            else:
                nextP = self.p * 0.7
        elif worstOver < AC.kSteerOvershootMin:
            self.good = self.p
            nextP = self.p * 1.3
        else:
            self.good = self.p
            self.done = True

        self.iteration += 1
        if self.iteration >= AC.kSteerVerifyMaxIterations:
            self.done = True
        if self.done:
            final = self.good if self.good is not None else (nextP or self.p)
            self.tuner.steerP = final
            self.tuner.applySteerP(final)
        else:
            self.p = nextP
            self.tuner.applySteerP(self.p)

    def isFinished(self) -> bool:
        return self.done or self.tuner.failed

    def end(self, interrupted: bool) -> None:
        if interrupted:
            self.tuner.fail("stopped during steering check")


class Autotuner:
    def __init__(self, drive) -> None:
        """:param drive: the EasySwerve DriveSubsystem"""
        self.drive = drive
        self.failed = False
        self.data: typing.Dict[str, typing.Dict[str, typing.Any]] = {}
        self.lines: typing.List[str] = []
        self.startGains = drive.gains
        self.newGains = drive.gains
        self.steerP = drive.gains.steerP
        self.steerKS = drive.gains.steerKS
        self.steerHistory: typing.List[typing.Tuple[float, float, float]] = []
        wpilib.SmartDashboard.putString("Autotune/Status", "idle")

    # ---- public: commands for the dashboard ----------------------------------

    def steeringCommand(self) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self._begin("steering"), *self._steeringSteps(), self._finish()
        ).withName("Autotune steering")

    def driveCommand(self) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self._begin("drive"), *self._driveSteps(), self._finish()
        ).withName("Autotune drive")

    def fullCommand(self) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self._begin("steering + drive"),
            *self._steeringSteps(),
            *self._driveSteps(),
            self._finish(),
        ).withName("Autotune all")

    def resetCommand(self) -> commands2.Command:
        """Forget saved tuning and go back to REV's default gains."""

        def run():
            p = G.reset()
            defaults = G.TunedGains()
            self.drive.applyGains(defaults)
            self.status(f"reset to defaults (deleted {p})")
            print(f"[autotune] reset to defaults: {defaults.summary()}")

        return commands2.InstantCommand(run).withName("Autotune reset")

    # ---- reporting ------------------------------------------------------------

    def status(self, text: str) -> None:
        wpilib.SmartDashboard.putString("Autotune/Status", text)
        print(f"[autotune] {text}")

    def report(self, line: str) -> None:
        self.lines.append(line)
        print(f"[autotune] {line}")

    def fail(self, reason: str) -> None:
        if self.failed:
            return
        self.failed = True
        self.report(f"FAILED: {reason}")
        self.status(f"FAILED: {reason}")
        # undo any trial gains
        self.drive.applyGains(self.startGains)

    # ---- building blocks ------------------------------------------------------

    def _guard(self, cmd: commands2.Command) -> commands2.Command:
        return cmd.onlyIf(lambda: not self.failed)

    def _begin(self, what: str) -> commands2.Command:
        def run():
            self.failed = False
            self.data = {}
            self.lines = []
            self.steerHistory = []
            self.startGains = self.drive.gains
            self.newGains = self.drive.gains
            self.steerP = self.drive.gains.steerP
            self.steerKS = self.drive.gains.steerKS
            if AC.kRequireTestMode and not wpilib.DriverStation.isTest():
                self.fail("switch the Driver Station to Test mode and enable first")
                return
            if not self.drive.steeringEncodersReady():
                self.fail("steering encoders not read yet; check their wiring and zeroing")
                return
            self.report(f"==== EasySwerve autotune: {what} ====")
            self.report(f"starting gains: {self.startGains.summary()}")

        return commands2.InstantCommand(run)

    def _rest(self) -> commands2.Command:
        return self._guard(commands2.WaitCommand(AC.kRestSeconds))

    def _steeringSteps(self) -> typing.List[commands2.Command]:
        r, s = AC.kSteerRampVoltsPerSecond, AC.kSteerStepVolts
        return [
            self._guard(_VoltageRun(self, STEER, "ramp+", lambda t: r * t, AC.kSteerRampSeconds)),
            self._rest(),
            self._guard(_VoltageRun(self, STEER, "ramp-", lambda t: -r * t, AC.kSteerRampSeconds)),
            self._rest(),
            self._guard(_VoltageRun(self, STEER, "step+", lambda t: s, AC.kSteerStepSeconds)),
            self._rest(),
            self._guard(_VoltageRun(self, STEER, "step-", lambda t: -s, AC.kSteerStepSeconds)),
            self._rest(),
            self._guard(commands2.InstantCommand(self._analyzeSteering)),
            self._guard(_SteerCheckAndAdjust(self)),
            self._guard(commands2.InstantCommand(self._steeringDone)),
        ]

    def _driveSteps(self) -> typing.List[commands2.Command]:
        r, s, d = AC.kDriveRampVoltsPerSecond, AC.kDriveStepVolts, AC.kDriveMaxDistanceM
        return [
            self._guard(_AlignWheels(self)),
            self._guard(_VoltageRun(self, DRIVE, "ramp+", lambda t: r * t, AC.kDriveRampMaxSeconds, d)),
            self._rest(),
            self._guard(_VoltageRun(self, DRIVE, "ramp-", lambda t: -r * t, AC.kDriveRampMaxSeconds, d)),
            self._rest(),
            self._guard(_VoltageRun(self, DRIVE, "step+", lambda t: s, AC.kDriveStepMaxSeconds, d)),
            self._rest(),
            self._guard(_VoltageRun(self, DRIVE, "step-", lambda t: -s, AC.kDriveStepMaxSeconds, d)),
            self._rest(),
            self._guard(commands2.InstantCommand(self._analyzeDrive)),
        ]

    def _finish(self) -> commands2.Command:
        def run():
            if self.failed:
                print("[autotune] gains unchanged:", self.drive.gains.summary())
                return
            g = dataclasses.replace(self.newGains, source="autotune")
            g.notes = {"report": list(self.lines)}
            path = G.save(g)
            self.drive.applyGains(g)
            self.report(f"final gains:    {g.summary()}")
            self.report(f"saved to {path} (loaded automatically at every startup)")
            self.status("done - gains saved and applied")
            sd = wpilib.SmartDashboard
            sd.putNumber("Autotune/drive P", g.driveP)
            sd.putNumber("Autotune/drive kS", g.driveKS)
            sd.putNumber("Autotune/drive kV", g.driveKV)
            sd.putNumber("Autotune/steer P", g.steerP)

        return commands2.InstantCommand(run)

    def applySteerP(self, p: float) -> None:
        self.drive.applyGains(
            dataclasses.replace(self.drive.gains, steerP=p, steerKS=self.steerKS)
        )

    # ---- analysis -------------------------------------------------------------

    def _fitAxis(self, axis: str) -> typing.Optional[typing.Dict[str, fit.FitResult]]:
        runsByLabel = self.data.get(axis, {})
        names = [name for name, _ in self.drive.modules]
        results = {}
        for name in names:
            runs = [samples[name] for samples, _ in runsByLabel.values()]
            results[name] = fit.fitFeedforward(runs)
            self.report(f"  {name}: {results[name].describe()}")
        bad = [n for n, r in results.items() if not r.ok]
        if bad:
            self.fail(f"{axis} fit failed for {', '.join(bad)}: {results[bad[0]].reason}")
            return None

        for key in ("kS", "kV"):
            med = fit.median([getattr(r, key) for r in results.values()])
            for n, r in results.items():
                v = getattr(r, key)
                if med > 0 and abs(v - med) / med > AC.kOutlierFraction:
                    self.report(
                        f"  WARNING: {n} {key}={v:.3f} is far from the others ({med:.3f}); "
                        "that module may be binding, rubbing or loose"
                    )
        return results

    def _analyzeSteering(self) -> None:
        self.report("steering model fits (volts, rad):")
        results = self._fitAxis(STEER)
        if results is None:
            return
        kS = fit.median([r.kS for r in results.values()])
        kV = fit.median([r.kV for r in results.values()])
        kA = fit.median([r.kA for r in results.values()])
        nominal = ModuleConstants.kNominalVoltage
        self.steerP = fit.steeringP(kV, kA, AC.kSteerDampingRatio, nominal)
        self.steerKS = AC.kSteerFrictionFraction * kS
        self.steerFrictionKS = kS
        self.report(
            f"  median kS={kS:.4f} kV={kV:.4f} kA={kA:.5f} -> starting P={self.steerP:.3f} "
            f"(damping ratio {AC.kSteerDampingRatio}), friction push {self.steerKS:.3f} V"
        )
        self.report("steering check-and-adjust:")

    def _steeringDone(self) -> None:
        self.newGains = dataclasses.replace(
            self.newGains, steerP=self.steerP, steerKS=self.steerKS
        )
        nominal = ModuleConstants.kNominalVoltage
        withoutPush = math.degrees(self.steerFrictionKS / (nominal * self.steerP))
        withPush = math.degrees(
            max(
                (self.steerFrictionKS - self.steerKS) / (nominal * self.steerP),
                math.radians(ModuleConstants.kSteerFrictionDeadbandDeg),
            )
        )
        self.report(
            f"  steering P = {self.steerP:.3f}, friction push = {self.steerKS:.3f} V "
            f"(worst holding error about {withPush:.2f} deg; {withoutPush:.2f} deg without the push)"
        )

    def _analyzeDrive(self) -> None:
        self.report("drive model fits (volts, m):")
        results = self._fitAxis(DRIVE)
        if results is None:
            return
        kS = fit.median([r.kS for r in results.values()])
        kV = fit.median([r.kV for r in results.values()])
        kA = fit.median([r.kA for r in results.values()])

        expectedKV = 12.0 / ModuleConstants.kDriveWheelFreeSpeedRps
        if abs(kV - expectedKV) / expectedKV > 0.4:
            self.report(
                f"  WARNING: kV={kV:.3f} but the gearing predicts about {expectedKV:.3f}; "
                "check wheel diameter, gear ratio and current limit"
            )

        # How far the SPARK's speed reading lags the true speed (its filter)
        delays = []
        stepRun = self.data[DRIVE].get("step+")
        if stepRun is not None:
            samples, measured = stepRun
            for name, _ in self.drive.modules:
                iv = fit.velocities(samples[name])
                true = [v for _, _, v in iv]
                meas = measured[name][1:]  # reading at the end of each interval
                dts = sorted(dt for dt, _, _ in iv)
                dt = dts[len(dts) // 2] if dts else 0.02
                delays.append(fit.estimateDelay(true, meas, dt))
        delay = fit.median(delays) if delays else 0.05
        p = fit.driveP(kA, delay, AC.kDriveSmoothing, ModuleConstants.kNominalVoltage)

        self.report(
            f"  median kS={kS:.4f} kV={kV:.4f} kA={kA:.5f}; speed reading lag {1000 * delay:.0f} ms"
        )
        self.report(f"  drive P = {p:.4f} (SIMC rule), feedforward kS={kS:.3f} V, kV={kV:.3f} V/(m/s)")
        self.newGains = dataclasses.replace(self.newGains, driveP=p, driveKS=kS, driveKV=kV)
