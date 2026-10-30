"""
Ziegler-Nichols steering tuning (the "ultimate gain" method), run on the
SPARK MAXes' own 1 ms steering loops.

The classic recipe, done automatically for all four modules at once:
  1. Use only P (I = D = 0).
  2. Make a small steering step and watch the wheel settle.
  3. Raise P a little and repeat, until the wheel no longer settles but keeps
     oscillating back and forth by the same amount. That P is the ultimate
     gain Ku; the time for one back-and-forth is the ultimate period Tu.
  4. Ziegler and Nichols' table turns Ku and Tu into P, I and D
     (examples/00_07_zieglernichols does this step, so you can see it).

The oscillations are fast (tens of milliseconds), so while the test runs each
SPARK sends its steering angle every 5 ms and the robot code reads it every
5 ms (robot.py calls sample() with addPeriodic). In the simulator, physics.py
records the 5 ms readings instead.

Run it with the wheels on the floor, so the steering feels the same load it
will in a match. The robot doesn't drive; the wheels only wiggle.
"""

import dataclasses
import math
import typing

import commands2
import wpilib

TAU = 2 * math.pi


@dataclasses.dataclass
class Trial:
    p: float  # the P tried (duty cycle per radian)
    decay: typing.Optional[float]  # amplitude ratio per half swing; None = no swing
    swingDeg: float  # size of the first overshoot, degrees
    periodS: typing.Optional[float]


@dataclasses.dataclass
class ModuleResult:
    name: str
    ku: typing.Optional[float] = None  # ultimate gain, duty cycle per radian
    tu: typing.Optional[float] = None  # ultimate period, seconds
    trials: typing.List[Trial] = dataclasses.field(default_factory=list)
    problem: str = ""


def _swings(times, errors, minSwingRad):
    """Split the error after the first overshoot into swings (between zero
    crossings). Returns (peak sizes, peak times) of the complete swings."""
    sign0 = math.copysign(1.0, errors[0]) if errors and errors[0] != 0 else 1.0
    # first crossing: the wheel has gone past the target
    start = None
    for i, e in enumerate(errors):
        if e * sign0 < 0:
            start = i
            break
    if start is None:
        return [], []
    peaks, peakTimes = [], []
    segSign = math.copysign(1.0, errors[start])
    best, bestT = 0.0, times[start]
    for i in range(start, len(errors)):
        e = errors[i]
        if e * segSign < 0:  # crossed again: this swing is complete
            peaks.append(best)
            peakTimes.append(bestT)
            segSign = -segSign
            best, bestT = 0.0, times[i]
        if abs(e) > best:
            best, bestT = abs(e), times[i]
    # ignore swings smaller than the sensor can really see
    keep = [(p, t) for p, t in zip(peaks, peakTimes) if p >= minSwingRad]
    return [p for p, _ in keep], [t for _, t in keep]


def analyzeStep(times, readings, target, minSwingDeg=0.3) -> Trial:
    """How one step response swings: decay ratio per half swing and period."""
    errors = [target - x for x in readings]
    peaks, peakTimes = _swings(times, errors, math.radians(minSwingDeg))
    if len(peaks) < 3:
        return Trial(p=0.0, decay=None, swingDeg=math.degrees(peaks[0]) if peaks else 0.0, periodS=None)
    n = len(peaks) - 1
    decay = (peaks[-1] / peaks[0]) ** (1.0 / n)
    # a whole period = two half swings
    period = 2.0 * (peakTimes[-1] - peakTimes[0]) / n
    return Trial(p=0.0, decay=decay, swingDeg=math.degrees(peaks[0]), periodS=period)


class ZieglerNicholsTest(commands2.Command):
    """Finds Ku and Tu for every steering module. When it ends, results holds
    one ModuleResult per module (FL, FR, RL, RR) and onDone(results) is called."""

    def __init__(
        self,
        drivetrain,
        onDone: typing.Callable[[typing.List[ModuleResult]], None] = lambda r: None,
        startP: typing.Optional[float] = None,
        growth: float = 1.25,
        maxP: float = 60.0,
        stepDeg: float = 10.0,
        settleS: float = 0.3,
        recordS: float = 0.6,
    ) -> None:
        super().__init__()
        self.drive = drivetrain
        self.onDone = onDone
        self.startP = startP
        self.growth = growth
        self.maxP = maxP
        self.step = math.radians(stepDeg)
        self.settleS = settleS
        self.recordS = recordS
        self.maxDutyInStep = 0.4  # at most 40 % power at the start of a step
        self.results: typing.List[ModuleResult] = []
        self.addRequirements(drivetrain)

    # ---- called every 5 ms from robot.py (real robot) ------------------------------

    def sample(self) -> None:
        if wpilib.RobotBase.isSimulation():
            return  # physics.py records the 5 ms readings in the simulator
        now = wpilib.Timer.getFPGATimestamp()
        for _, m in self.drive.modules:
            log = getattr(m, "fineLog", None)
            if log is not None:
                log.append((now, self._read(m)))

    @staticmethod
    def _read(m) -> float:
        enc = m.turningEncoder
        if enc.location == "rio":
            return enc.relative.getPosition()  # NEO sensor, radians, not wrapped
        return enc.getPosition()  # 0..2pi, unwrapped in _unwrap

    @staticmethod
    def _unwrap(values):
        out, prev, turns = [], None, 0.0
        for v in values:
            if prev is not None:
                d = v - prev
                if d > math.pi:
                    turns -= TAU
                elif d < -math.pi:
                    turns += TAU
            out.append(v + turns)
            prev = v
        return out

    # ---- the test --------------------------------------------------------------------

    def initialize(self) -> None:
        self.startGains = self.drive.gains
        p0 = self.startP if self.startP is not None else self.startGains.steerP
        self.safeP = p0
        self.p = {name: p0 for name, _ in self.drive.modules}
        self.results = [ModuleResult(name) for name, _ in self.drive.modules]
        self.direction = 1.0
        self.aborted = ""
        for _, m in self.drive.modules:
            m.setFastSteeringSignals(True)
            m.setDrivingVoltage(0.0)
        self._startTrial()

    def _name(self, m) -> str:
        for name, mod in self.drive.modules:
            if mod is m:
                return name
        return ""

    def _stepFor(self, p: float) -> float:
        """Step size for a trial: small enough that P x step stays well under
        full power, so the motor isn't maxed out (Ziegler-Nichols assumes it
        isn't), but never under 2 degrees, which the sensor sees clearly."""
        return max(math.radians(2.0), min(self.step, self.maxDutyInStep / p))

    def _active(self):
        """(module, result) pairs still being tested."""
        return [
            (m, r) for (_, m), r in zip(self.drive.modules, self.results)
            if r.ku is None and not r.problem
        ]

    def _startTrial(self) -> None:
        now = wpilib.Timer.getFPGATimestamp()
        self.phase = "settle"
        self.phaseStart = now
        for (name, m), r in zip(self.drive.modules, self.results):
            testing = r.ku is None and not r.problem
            p = self.p[name] if testing else self.safeP
            m.applyGains(dataclasses.replace(self.startGains, steerP=p, steerI=0.0, steerD=0.0))
            m.turningVoltage = None
            m.fineLog = None
            m.znStart = self._read(m)  # hold where it is now
            m._steerTo(m.znStart % TAU)
        self.direction = -self.direction  # alternate step directions

    def execute(self) -> None:
        if self.aborted:
            return
        now = wpilib.Timer.getFPGATimestamp()
        if self.phase == "settle":
            if now - self.phaseStart >= self.settleS:
                self.phase = "record"
                self.phaseStart = now
                for m, _ in self._active():
                    m.znStart = self._read(m)
                    m.znTarget = m.znStart + self.direction * self._stepFor(self.p[self._name(m)])
                    m.fineLog = [(now, m.znStart)]
                    m._steerTo(m.znTarget % TAU)
            return

        # recording: watch for a runaway (a backwards motor or encoder)
        for m, r in self._active():
            if now - self.phaseStart > 0.15 and m.fineLog:
                err = abs(m.znTarget - self._unwrap([x for _, x in m.fineLog])[-1])
                if err > math.radians(60):
                    self.aborted = (
                        f"{r.name} steering ran away from its target: a steering motor or "
                        "encoder is backwards. Run examples/00_03_steeronebyone first."
                    )
                    return
        if now - self.phaseStart < self.recordS:
            return

        for m, r in self._active():
            log = m.fineLog or []
            m.fineLog = None
            times = [t for t, _ in log]
            readings = self._unwrap([x for _, x in log])
            trial = analyzeStep(times, readings, m.znTarget)
            trial.p = self.p[r.name]
            r.trials.append(trial)
            if trial.decay is not None and trial.decay >= 1.0:
                r.ku, r.tu = self._ultimate(r)
            elif self.p[r.name] * self.growth > self.maxP:
                r.problem = f"never kept oscillating, even at P = {self.p[r.name]:.1f}"
            else:
                self.p[r.name] *= self.growth
        if self._active():
            self._startTrial()

    @staticmethod
    def _ultimate(r: ModuleResult):
        """Ku where the swings stop shrinking (decay = 1), interpolated between
        the last two trials (log scale), and the period there."""
        last = r.trials[-1]
        prev = r.trials[-2] if len(r.trials) >= 2 else None
        ku = last.p
        if prev is not None and prev.decay is not None and prev.decay < 1.0 < last.decay:
            a, b = math.log(prev.decay), math.log(last.decay)
            f = (0.0 - a) / (b - a)
            ku = math.exp(math.log(prev.p) + f * (math.log(last.p) - math.log(prev.p)))
        return ku, last.periodS

    def isFinished(self) -> bool:
        return bool(self.aborted) or not self._active()

    def end(self, interrupted: bool) -> None:
        for _, m in self.drive.modules:
            m.fineLog = None
            m.setFastSteeringSignals(False)
        self.drive.applyGains(self.startGains)  # back to the gains it started with
        for (_, m), r in zip(self.drive.modules, self.results):
            m._steerTo(self._read(m) % TAU)
            if r.ku is None and not r.problem:
                r.problem = self.aborted or ("stopped before finishing" if interrupted else "")
        self.onDone(self.results)


class SteeringStepCheck(commands2.Command):
    """Steers every wheel back and forth by stepDeg a few times with the gains
    the drivetrain has now, and measures overshoot and settling time.

    When it ends, onDone(report) is called with a dict:
      overshootPct  worst overshoot of any module, percent of the step
      settleS       slowest time to stay within 2 degrees, seconds (None = never)
      perModule     {name: (overshootPct, settleS)}
    """

    def __init__(self, drivetrain, onDone=lambda report: None, stepDeg: float = 45.0, steps: int = 4, stepS: float = 1.0) -> None:
        super().__init__()
        self.drive = drivetrain
        self.onDone = onDone
        self.step = math.radians(stepDeg)
        self.steps = steps
        self.stepS = stepS
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        self.n = 0
        self.worst = {name: (0.0, 0.0) for name, _ in self.drive.modules}
        self.base = {name: ZieglerNicholsTest._read(m) for name, m in self.drive.modules}
        for _, m in self.drive.modules:
            m.setDrivingVoltage(0.0)
            m.turningVoltage = None
        self._next()

    def _next(self) -> None:
        self.t0 = wpilib.Timer.getFPGATimestamp()
        sign = 1.0 if self.n % 2 == 0 else -1.0
        self.logs = {}
        for name, m in self.drive.modules:
            start = ZieglerNicholsTest._read(m)
            target = self.base[name] + (self.step if sign > 0 else 0.0)
            self.logs[name] = (start, target, [])
            m._steerTo(target % TAU)

    def execute(self) -> None:
        now = wpilib.Timer.getFPGATimestamp()
        for name, m in self.drive.modules:
            self.logs[name][2].append((now - self.t0, ZieglerNicholsTest._read(m)))
        if now - self.t0 < self.stepS:
            return
        for name, (start, target, log) in self.logs.items():
            xs = ZieglerNicholsTest._unwrap([x for _, x in log])
            span = target - start
            if abs(span) < 1e-6:
                continue
            progress = [(x - start) / span for x in xs]
            over = max(0.0, (max(progress) - 1.0) * 100.0)
            tol = math.radians(2.0) / abs(span)
            settle = None
            for i in range(len(progress)):
                if all(abs(p - 1.0) <= tol for p in progress[i:]):
                    settle = log[i][0]
                    break
            wo, ws = self.worst[name]
            ws = None if (settle is None or ws is None) else max(ws, settle)
            self.worst[name] = (max(wo, over), ws)
        self.n += 1
        if self.n < self.steps:
            self._next()

    def isFinished(self) -> bool:
        return self.n >= self.steps

    def end(self, interrupted: bool) -> None:
        overs = [o for o, _ in self.worst.values()]
        settles = [s for _, s in self.worst.values()]
        report = {
            "overshootPct": max(overs) if overs else 0.0,
            "settleS": None if any(s is None for s in settles) else max(settles),
            "perModule": dict(self.worst),
            "interrupted": interrupted,
        }
        self.onDone(report)
