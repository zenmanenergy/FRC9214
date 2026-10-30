"""
The math behind autotune. Pure Python (no robot, no numpy), so it is easy to
test and runs fine on the roboRIO.

1. fitFeedforward(): fits the motor model
       volts = kS * sign(velocity) + kV * velocity + kA * acceleration
   from recorded voltage and position samples. Same method WPILib's SysId
   uses: a least-squares fit of how each loop's velocity follows from the
   previous one and the applied voltage, converted to kS, kV, kA. No noisy
   differentiation for acceleration is needed.

2. estimateDelay(): how far the SPARK's own speed measurement lags behind
   the real speed (its filtering), found by a least-squares time shift.

3. Gain rules that turn the model into SPARK gains:
   - steering: P chosen for a target damping ratio of the position loop
   - driving: SIMC rule (Skogestad) for a first-order plant with delay
"""

import dataclasses
import math
import typing

Sample = typing.Tuple[float, float, float]  # (time s, volts, position)


@dataclasses.dataclass
class FitResult:
    kS: float
    kV: float
    kA: float
    r2: float
    rows: int
    ok: bool
    reason: str = ""

    def describe(self) -> str:
        return (
            f"kS={self.kS:.4f} kV={self.kV:.4f} kA={self.kA:.5f} "
            f"R2={self.r2:.4f} rows={self.rows}" + ("" if self.ok else f" FAILED: {self.reason}")
        )


def _solve(m: typing.List[typing.List[float]], b: typing.List[float]) -> typing.List[float]:
    """Solve a small square linear system by Gaussian elimination with pivoting."""
    n = len(b)
    a = [row[:] + [bi] for row, bi in zip(m, b)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-12:
            raise ValueError("singular system (not enough varied data)")
        a[col], a[piv] = a[piv], a[col]
        for r in range(n):
            if r != col:
                f = a[r][col] / a[col][col]
                for c in range(col, n + 1):
                    a[r][c] -= f * a[col][c]
    return [a[i][n] / a[i][i] for i in range(n)]


def velocities(run: typing.Sequence[Sample]) -> typing.List[typing.Tuple[float, float, float]]:
    """Turn (t, V, position) samples into (dt, V, velocity) per interval.

    Entry k describes the interval from sample k to k+1: the voltage that was
    applied during it (commanded at sample k) and the average velocity over it.
    """
    out = []
    for k in range(len(run) - 1):
        t0, v0, p0 = run[k]
        t1, _, p1 = run[k + 1]
        dt = t1 - t0
        if dt <= 0:
            continue
        out.append((dt, v0, (p1 - p0) / dt))
    return out


def fitFeedforward(
    runs: typing.Sequence[typing.Sequence[Sample]],
    minVelocityFraction: float = 0.02,
) -> FitResult:
    """Fit kS, kV, kA from one or more recorded runs.

    :param runs: each run is a list of (time, volts, position) samples taken at
                 a steady loop period (20 ms). Mix slow ramps and fast steps,
                 both directions, for a good fit.
    :param minVelocityFraction: rows slower than this fraction of the fastest
                 speed seen are dropped, because sign(velocity) is unreliable
                 near zero.
    """
    # Velocities here are averages over each loop interval (position
    # difference / dt). For a first-order motor with constant voltage during
    # each interval, consecutive interval averages obey exactly:
    #     vB = alpha*vA + b1*VA + b2*VB + gamma*sign(vA)
    # where VA, VB are the voltages applied during the two intervals. Then
    #     kV = (1 - alpha)/(b1 + b2),  kS = -gamma/(b1 + b2),
    #     kA = kV * (-dt / ln(alpha))      (time constant kA/kV = -dt/ln(alpha))
    # Encoder noise makes plain least squares biased here: vA and vB share a
    # position sample, so their errors are correlated. An instrumental-variable
    # fit fixes that: vA's instrument is the velocity two intervals earlier,
    # whose noise is independent of both vA's and vB's. Voltages and the sign
    # term are their own instruments.
    rows = []  # (vPrev2, vA, VA, VB, vB, dts...)
    allDt = []
    vmax = 0.0
    for run in runs:
        iv = velocities(run)
        for k in range(2, len(iv) - 1):
            dtZ, _, vZ = iv[k - 2]
            dtA, voltsA, vA = iv[k]
            dtB, voltsB, vB = iv[k + 1]
            rows.append((vZ, vA, voltsA, voltsB, vB, (dtZ, iv[k - 1][0], dtA, dtB)))
            allDt.append(dtB)
            vmax = max(vmax, abs(vA), abs(vB))

    if len(rows) < 30 or vmax <= 0:
        return FitResult(0, 0, 0, 0, len(rows), False, "not enough moving data")

    allDt.sort()
    dtMed = allDt[len(allDt) // 2]
    thresh = minVelocityFraction * vmax

    n = 4
    ztx = [[0.0] * n for _ in range(n)]
    zty = [0.0] * n
    used = []
    for vZ, vA, voltsA, voltsB, vB, dts in rows:
        if abs(vA) < thresh or abs(vZ) < thresh or (vZ > 0) != (vA > 0):
            continue
        if any(abs(d - dtMed) > 0.25 * dtMed for d in dts):
            continue  # skip loop overruns
        sgn = 1.0 if vA > 0 else -1.0
        x = (vA, voltsA, voltsB, sgn)
        z = (vZ, voltsA, voltsB, sgn)
        for i in range(n):
            zty[i] += z[i] * vB
            for j in range(n):
                ztx[i][j] += z[i] * x[j]
        used.append((x, vB))

    if len(used) < 30:
        return FitResult(0, 0, 0, 0, len(used), False, "not enough moving data")

    try:
        alpha, b1, b2, gamma = _solve(ztx, zty)
    except ValueError as e:
        return FitResult(0, 0, 0, 0, len(used), False, str(e))
    beta = b1 + b2

    mean = sum(y for _, y in used) / len(used)
    ssTot = sum((y - mean) ** 2 for _, y in used)
    ssRes = sum(
        (y - (alpha * x[0] + b1 * x[1] + b2 * x[2] + gamma * x[3])) ** 2 for x, y in used
    )
    r2 = 1.0 - ssRes / ssTot if ssTot > 0 else 0.0

    if not (0.0 < alpha < 1.0) or beta <= 0:
        return FitResult(
            0, 0, 0, r2, len(used), False,
            f"model not physical (alpha={alpha:.4f}, beta={beta:.5f}); "
            "check motor inversion and that the mechanism moved freely",
        )

    kV = (1.0 - alpha) / beta
    kA = kV * (-dtMed / math.log(alpha))
    kS = -gamma / beta
    ok = r2 >= 0.9 and kV > 0 and kA > 0
    reason = "" if ok else f"poor fit (R2={r2:.3f}); check for binding, slipping or loose wiring"
    return FitResult(kS, kV, kA, r2, len(used), ok, reason)


def estimateDelay(
    trueVelocity: typing.Sequence[float],
    measuredVelocity: typing.Sequence[float],
    dt: float,
    maxDelay: float = 0.3,
) -> float:
    """Seconds by which measuredVelocity lags trueVelocity.

    trueVelocity comes from position differences: entry k is the average over
    interval k, i.e. the speed at the middle of that interval. measuredVelocity
    entry k is what the SPARK reported at the END of interval k (the reading its
    own speed loop uses). The delay is the shift (searched in quarter-sample
    steps, with linear interpolation) that best lines the two up in the
    least-squares sense.
    """
    n = min(len(trueVelocity), len(measuredVelocity))
    if n < 20:
        return 0.05
    a = list(trueVelocity[:n])
    b = list(measuredVelocity[:n])

    def shifted(k: float) -> typing.Optional[float]:
        if k < 0 or k > n - 1:
            return None
        i = int(math.floor(k))
        f = k - i
        return a[i] if i + 1 >= n else a[i] * (1 - f) + a[i + 1] * f

    best, bestShift = math.inf, 0.0
    steps = int(maxDelay / dt * 4)
    for q in range(0, steps + 1):
        shift = q / 4.0  # in samples
        err, count = 0.0, 0
        for k in range(n):
            # measured[k] (end of interval k) vs true speed `shift` samples
            # earlier than that instant; true[j] is centered at j + 0.5
            v = shifted(k + 0.5 - shift)
            if v is None:
                continue
            err += (b[k] - v) ** 2
            count += 1
        if count >= n // 2 and err / count < best:
            best, bestShift = err / count, shift
    return bestShift * dt


def steeringP(
    kV: float, kA: float, zeta: float = 0.75, nominalVolts: float = 12.0,
    minP: float = 0.05, maxP: float = 8.0,
) -> float:
    """P gain (duty per radian) giving the target damping ratio.

    Position loop with P only: kA*s^2 + kV*s + Kp = 0, so
    zeta = kV / (2*sqrt(kA*Kp))  ->  Kp = kV^2 / (4 * zeta^2 * kA)  [volts/rad].
    """
    kpVolts = kV * kV / (4.0 * zeta * zeta * kA)
    return max(minP, min(maxP, kpVolts / nominalVolts))


def driveP(
    kA: float, delay: float, smoothing: float = 1.5, nominalVolts: float = 12.0,
    minP: float = 0.0, maxP: float = 1.0,
) -> float:
    """P gain (duty per m/s) from the SIMC rule for a first-order plant with
    delay theta, with closed-loop time constant tau_c = smoothing * theta:

        Kc = (1/k) * tau / (tau_c + theta),  k = 1/kV, tau = kA/kV
           = kA / ((smoothing + 1) * theta)   [volts per m/s]

    smoothing 1.0 is SIMC's "tight" setting (about 8 % overshoot),
    1.5 about 4 %, 2.0 about 1 %.
    """
    theta = max(0.01, delay)
    kcVolts = kA / ((smoothing + 1.0) * theta)
    return max(minP, min(maxP, kcVolts / nominalVolts))


def median(values: typing.Sequence[float]) -> float:
    s = sorted(values)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def stepResponse(
    times: typing.Sequence[float], angles: typing.Sequence[float], start: float, target: float
) -> typing.Tuple[float, float]:
    """Overshoot (fraction of step size) and settle time (s, within 2 degrees)
    for one recorded steering step. angles are unwrapped radians."""
    step = target - start
    if abs(step) < 1e-6 or not times:
        return 0.0, 0.0
    direction = 1.0 if step > 0 else -1.0
    peak = max((a - target) * direction for a in angles)
    overshoot = max(0.0, peak) / abs(step)
    tol = math.radians(2.0)
    settle = times[-1] - times[0]
    for i in range(len(angles)):
        if all(abs(a - target) <= tol for a in angles[i:]):
            settle = times[i] - times[0]
            break
    return overshoot, settle
