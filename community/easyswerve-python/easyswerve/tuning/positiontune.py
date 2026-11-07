"""
Position autotune: finds the PID gains that drive the robot to a spot on the
field (posP/posI/posD) and turn it to face a direction (headP/headI/headD).
These are the gains driveDistance, turnTo, heading hold, driveTo and the
PathPlanner autos use.

Run it after the steering and drive autotunes (setup steps 7 and 8), on the
floor, in Test mode, starting at one end of the practice carpet. Its check
moves are 1 m, or less to fit the carpet (robot_settings.py), and it turns
on the spot.
It takes about a minute. For each of the two (heading first, then position):

  1. Measure the robot. Ask for a sudden speed (0.6 m/s forward, or a spin)
     and record how the robot follows: how long before it starts to move
     (the dead time) and how long it takes to get up to speed (the lag).
     The SIMC rules turn those two numbers into P, I and D.

  2. Ziegler-Nichols. Use only P, make small steps and raise P until the
     robot rocks back and forth by the same amount every swing. That P is
     the ultimate gain Ku and one rock back and forth is the period Tu.
     Ziegler and Nichols' "no overshoot" row turns them into P, I and D.

  3. Try both. Each set of gains does real moves (1 m forward and back, or
     less on a short carpet,
     90 degrees and back), and the code measures how close the robot stops
     to the target and how far it rolls past it. The set that stops closest
     is kept; if both stop about as close (within a quarter of the pass
     limit), the one that rolls past less. It must stop within 2 cm
     (1.5 degrees) to pass.

The robot code's own loop runs every 20 ms, so this is tuned on the loop
that really controls the position: the roboRIO's, through the SPARK MAXes'
speed loops (which the drive autotune tuned).

Safety: disabling stops it at once, and a test stops if the robot gets far
from where it should be (40 cm or 45 degrees).
"""

import dataclasses
import math
import typing

import commands2
import wpilib
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

from .. import carpet
from ..drivetrain.constants import DriveConstants
from ..drivetrain.positioncontrol import PositionController, headingPID
from .zieglernichols import _swings

# Ziegler and Nichols' "no overshoot" row: P = 0.2 Ku, Ti = Tu / 2, Td = Tu / 3
ZN_KP, ZN_TI, ZN_TD = 0.20, 0.50, 0.33


def _now() -> float:
    return wpilib.Timer.getFPGATimestamp()


class _Abort(Exception):
    pass


@dataclasses.dataclass
class _Axis:
    name: str  # "heading" or "position"
    unit: str  # for printing
    scale: float  # multiply to print in that unit
    stepSpeed: float  # the speed of the model test (m/s or rad/s)
    znStepMax: float  # Ziegler-Nichols step size, at most (m or rad)
    znStepMin: float  # and at least
    znSpeedLimit: float  # never ask for more speed than this (m/s or rad/s)
    minSwing: float  # swings smaller than this are sensor noise
    runaway: float  # stop if the error gets this big
    move: float  # size of the check moves
    passError: float  # a good set stops this close
    maxOvershoot: float  # and rolls past by no more than this
    maxP: float  # Ziegler-Nichols gives up above this P


# Never ask for more than MAX_SPEED_MPS (the wheels are limited to it: a test
# that hits the limit measures the limit, not the robot). Spinning, a wheel
# moves at (turn rate x its distance from the center).
_MPS = DriveConstants.kMaxSpeedMetersPerSecond
_WHEEL_R = math.hypot(DriveConstants.kTrackWidth / 2, DriveConstants.kWheelBase / 2)

HEADING = _Axis(
    "heading", "deg", math.degrees(1.0), min(2.0, 0.5 * _MPS / _WHEEL_R), math.radians(8), math.radians(2),
    min(6.0, 0.9 * _MPS / _WHEEL_R), math.radians(0.3), math.radians(45), math.radians(90),
    math.radians(1.5), math.radians(8), 80.0,
)
POSITION = _Axis(
    "position", "cm", 100.0, min(0.6, 0.5 * _MPS), 0.05, 0.01,
    min(2.0, 0.9 * _MPS), 0.003, 0.40, max(0.4, min(1.0, carpet.END_AHEAD_M - 0.1)),  # check move fits the carpet
    0.02, 0.08, 80.0,
)


def fitLag(times, values, speed):
    """Fit the response to a sudden speed step: the robot starts moving after
    a dead time, then gets up to speed with a lag (first order). Returns
    (gain, dead time s, lag s), found by trying every pair on a 5 ms grid."""
    t0, x0 = times[0], values[0]
    ts = [t - t0 for t in times]
    xs = [x - x0 for x in values]
    best = None
    for i in range(0, 41):  # dead time 0 to 200 ms
        dead = 0.005 * i
        for j in range(1, 81):  # lag 5 to 400 ms
            lag = 0.005 * j
            shape = []
            for t in ts:
                s = t - dead
                shape.append(speed * (s - lag * (1.0 - math.exp(-s / lag))) if s > 0 else 0.0)
            den = sum(f * f for f in shape)
            if den <= 0:
                continue
            gain = sum(f * x for f, x in zip(shape, xs)) / den  # best gain for this pair
            err = sum((x - gain * f) ** 2 for f, x in zip(shape, xs))
            if best is None or err < best[0]:
                best = (err, gain, dead, lag)
    return best[1], best[2], best[3]


def simcPID(gain: float, dead: float, lag: float):
    """SIMC rules (Skogestad) for something that integrates (speed in,
    position out) with a dead time and a lag. tc is how fast the loop should
    respond: as fast as the dead time allows, but never under 0.15 s, so the
    wheels don't fight each other or slip."""
    tc = max(dead, 0.15)
    kc = 1.0 / (gain * (tc + dead))
    ti = 4.0 * (tc + dead)
    td = lag
    # from the "series" form of the rules to WPILib's P, I, D
    return kc * (1.0 + td / ti), kc / ti, kc * td


def znPID(ku: float, tu: float):
    p = ZN_KP * ku
    return p, p / (ZN_TI * tu), p * (ZN_TD * tu)


class PositionTune(commands2.Command):
    """The whole position autotune, as one command (it steps through the
    tests one loop at a time)."""

    def __init__(self, tuner) -> None:
        super().__init__()
        self.tuner = tuner
        self.drive = tuner.drive
        self.addRequirements(self.drive)

    # ---- the command -----------------------------------------------------------

    def initialize(self) -> None:
        if self.tuner.newGains is not self.drive.gains:
            self.drive.applyGains(self.tuner.newGains)  # use the drive autotune's gains
        self.gains = self.drive.gains
        self.done = False
        self.steps = self._run()

    def execute(self) -> None:
        if self.done:
            return
        try:
            next(self.steps)
        except StopIteration:
            self.done = True
        except _Abort as e:
            self._stop()
            self.tuner.fail(str(e))
            self.done = True

    def isFinished(self) -> bool:
        return self.done or self.tuner.failed

    def end(self, interrupted: bool) -> None:
        self._stop()
        if interrupted:
            self.tuner.fail("stopped (robot disabled or command interrupted)")

    # ---- reading and driving -----------------------------------------------------

    def _stop(self) -> None:
        self.drive.driveSpeeds(ChassisSpeeds())

    def _value(self, axis: _Axis) -> float:
        """Where the robot is along the axis being tuned: meters ahead of
        where the test started, or radians turned (counting past 180)."""
        pose = self.drive.getPose()
        if axis is POSITION:
            dx, dy = pose.X() - self.origin.X(), pose.Y() - self.origin.Y()
            a = self.origin.rotation()
            return dx * a.cos() + dy * a.sin()
        turned = (pose.rotation() - self.origin.rotation()).radians()
        self.turned += math.remainder(turned - self.turned, math.tau)  # unwrap
        return self.turned

    def _push(self, axis: _Axis, speed: float) -> None:
        """Drive at a speed along the axis (m/s forward, or rad/s spinning)."""
        if axis is POSITION:
            # keep facing the start direction while driving
            heading = self.drive.getPose().rotation().radians()
            omega = self.keepHeading.calculate(heading, self.origin.rotation().radians())
            self.drive.driveSpeeds(ChassisSpeeds(speed, 0.0, omega))
        else:
            self.drive.driveSpeeds(ChassisSpeeds(0.0, 0.0, speed))

    def _wait(self, seconds: float, axis: typing.Optional[_Axis] = None, speed: float = 0.0):
        t0 = _now()
        while _now() - t0 < seconds:
            if axis is None:
                self._stop()
            else:
                self._push(axis, speed)
            yield

    def _start(self, axis: _Axis):
        """Point the wheels for this test (a crawl), stop, and remember the start."""
        self.tuner.status(f"position autotune: {axis.name}, getting ready")
        yield from self._wait(0.5, axis, 0.03 if axis is POSITION else 0.1)
        yield from self._wait(0.5)
        self.origin = self.drive.getPose()
        self.turned = 0.0
        self.keepHeading = headingPID(self.gains)

    # ---- the tests -------------------------------------------------------------------

    def _run(self):
        report = self.tuner.report
        report("==== position autotune ====")
        newGains = self.gains
        for axis in (HEADING, POSITION):
            yield from self._start(axis)
            model = yield from self._model(axis)
            zn = yield from self._zieglerNichols(axis)
            candidates = [("model (SIMC)", model)]
            if zn is not None:
                candidates.append(("Ziegler-Nichols", zn))
            best = None
            for label, pid in candidates:
                score = yield from self._check(axis, label, pid, newGains)
                if best is None or self._better(axis, score, best[0]):
                    best = (score, label, pid)
            score, label, (p, i, d) = best
            if score[0] > 0:
                report(f"  WARNING: no {axis.name} gains stopped within "
                       f"{axis.passError * axis.scale:.1f} {axis.unit}. Keeping the best ones.")
                report("  Check the wheel diameter (setup step 10), the drive autotune,")
                report("  and for slipping wheels or a dirty floor.")
            report(f"  {axis.name}: keeping {label}: P={p:.3f} I={i:.3f} D={d:.4f}")
            if axis is HEADING:
                newGains = dataclasses.replace(newGains, headP=p, headI=i, headD=d)
            else:
                newGains = dataclasses.replace(newGains, posP=p, posI=i, posD=d)
            self.drive.applyGains(newGains)
            self.gains = newGains
        self.tuner.newGains = newGains
        report("  driveDistance, turnTo and heading hold use these right away;")
        report("  driveTo and the PathPlanner autos after the robot code restarts.")

    def _model(self, axis: _Axis):
        """Step test: a sudden speed forward, then back. Fit dead time and lag."""
        self.tuner.status(f"position autotune: measuring {axis.name} response")
        fits = []
        for sign in (1.0, -1.0):
            times, values = [_now()], [self._value(axis)]
            t0 = times[0]
            while _now() - t0 < 0.7:
                self._push(axis, sign * axis.stepSpeed)
                yield
                times.append(_now())
                values.append(self._value(axis))
            yield from self._wait(0.6)
            moved = abs(values[-1] - values[0])
            if moved < 0.2 * axis.stepSpeed * 0.7:
                raise _Abort(f"the robot hardly moved in the {axis.name} test "
                             f"({moved * axis.scale:.1f} {axis.unit}); check the drive autotune")
            fits.append(fitLag(times, values, sign * axis.stepSpeed))
        gain = sum(f[0] for f in fits) / len(fits)
        dead = sum(f[1] for f in fits) / len(fits)
        lag = sum(f[2] for f in fits) / len(fits)
        if gain < 0:
            raise _Abort(f"the robot moved the wrong way in the {axis.name} test: "
                         "check the gyro and drive motor directions (setup steps 6 and 9)")
        p, i, d = simcPID(gain, dead, lag)
        self.tuner.report(
            f"  {axis.name} model: follows {100 * gain:.0f} % of the speed asked, dead time "
            f"{1000 * dead:.0f} ms, lag {1000 * lag:.0f} ms -> P={p:.3f} I={i:.3f} D={d:.4f}"
        )
        return p, i, d

    def _zieglerNichols(self, axis: _Axis):
        """Raise P until the robot keeps rocking back and forth."""
        self.tuner.status(f"position autotune: Ziegler-Nichols on {axis.name}")
        p, trials, direction = 2.0, [], 1.0
        while True:
            step = max(axis.znStepMin, min(axis.znStepMax, 0.5 * axis.znSpeedLimit / p))
            base = self._value(axis)
            target = base + direction * step
            direction = -direction
            times, errors = [], []
            t0 = _now()
            while _now() - t0 < 1.5:
                err = target - self._value(axis)
                if abs(err) > axis.runaway:
                    raise _Abort(f"the robot ran away from its {axis.name} target: "
                                 "check the gyro and drive motor directions")
                speed = max(-axis.znSpeedLimit, min(axis.znSpeedLimit, p * err))
                self._push(axis, speed)
                times.append(_now())
                errors.append(err)
                yield
            yield from self._wait(0.3)
            peaks, peakTimes = _swings(times, errors, axis.minSwing)
            decay, period = None, None
            if len(peaks) >= 3:
                n = len(peaks) - 1
                decay = (peaks[-1] / peaks[0]) ** (1.0 / n)
                period = 2.0 * (peakTimes[-1] - peakTimes[0]) / n
            trials.append((p, decay, period))
            if decay is not None and decay >= 1.0:
                ku = p
                if len(trials) >= 2 and trials[-2][1] is not None and trials[-2][1] < 1.0:
                    prevP, prevDecay = trials[-2][0], trials[-2][1]
                    a, b = math.log(prevDecay), math.log(decay)
                    f = -a / (b - a)
                    ku = math.exp(math.log(prevP) + f * (math.log(p) - math.log(prevP)))
                pid = znPID(ku, period)
                self.tuner.report(
                    f"  {axis.name} Ziegler-Nichols: Ku={ku:.2f} Tu={1000 * period:.0f} ms "
                    f"-> P={pid[0]:.3f} I={pid[1]:.3f} D={pid[2]:.4f}"
                )
                return pid
            if p * 1.25 > axis.maxP:
                self.tuner.report(f"  {axis.name} Ziegler-Nichols: never kept rocking, "
                                  f"even at P={p:.0f}; using the model only")
                return None
            p *= 1.25

    @staticmethod
    def _better(axis: _Axis, a, b) -> bool:
        """Is score a better than score b? (failed?, stopping error, roll-past)"""
        if a[0] != b[0]:
            return a[0] < b[0]  # passing beats failing
        if abs(a[1] - b[1]) > axis.passError / 4:
            return a[1] < b[1]  # clearly closer
        return a[2] < b[2]  # about as close: the one that rolls past less

    def _check(self, axis: _Axis, label: str, pid, gains):
        """Real moves with a set of gains: out and back. Returns a score
        (failed?, average stopping error, worst roll-past) to compare."""
        self.tuner.status(f"position autotune: checking {axis.name} {label}")
        p, i, d = pid
        if axis is HEADING:
            gains = dataclasses.replace(gains, headP=p, headI=i, headD=d)
        else:
            gains = dataclasses.replace(gains, posP=p, posI=i, posD=d)
        yield from self._start(axis)
        stops, overs = [], []
        for goal in (axis.move, 0.0):
            start = self._value(axis)
            direction = 1.0 if goal > start else -1.0
            if axis is POSITION:
                a = self.origin.rotation()
                target = Pose2d(self.origin.X() + goal * a.cos(), self.origin.Y() + goal * a.sin(), a)
            else:
                target = Pose2d(self.origin.translation(), self.origin.rotation() + Rotation2d(goal))
            control = PositionController(
                gains, DriveConstants.kMaxSpeedMetersPerSecond * 0.5,
                DriveConstants.kMaxAngularSpeed * 0.5, DriveConstants.kDistanceAccelMps2,
            )
            t0, over, late = _now(), 0.0, []
            while _now() - t0 < 3.0:
                pose = self.drive.getPose()
                vx, vy, omega = control.calculate(pose, target)
                self.drive.driveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, pose.rotation()))
                yield
                err = goal - self._value(axis)
                if abs(err) > abs(axis.move) + axis.runaway:
                    raise _Abort(f"the robot ran away during the {axis.name} check")
                over = max(over, -direction * err)
                if _now() - t0 > 2.6:
                    late.append(abs(err))  # the last 0.4 s: where it stopped
            stops.append(sum(late) / max(len(late), 1))
            overs.append(over)
        yield from self._wait(0.3)
        stop, over = sum(stops) / len(stops), max(overs)
        failed = stop > axis.passError or over > axis.maxOvershoot
        self.tuner.report(
            f"  {axis.name} check, {label}: stops {stop * axis.scale:.2f} {axis.unit} off, "
            f"rolls past {over * axis.scale:.1f} {axis.unit} -> {'not good enough' if failed else 'good'}"
        )
        return (1 if failed else 0, stop, over)
