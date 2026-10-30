"""
Autotune tests.

The first group checks the math alone (no robot needed). The last test runs
the whole autotune in the simulator, where physics.py's "true" motor
constants are known, and checks that autotune finds them.

    python -m robotpy test
"""

import math
import random

import pytest

from easyswerve.tuning import fit

from easyswerve.sim.physics import findDrivetrain


# ---- math only ----------------------------------------------------------------


def _plant(kS, kV, kA, profile, seconds, dt=0.02, sub=40):
    """Exact-ish simulation of V = kS*sign(v) + kV*v + kA*a, sampled every dt."""
    v = p = t = 0.0
    out = []
    h = dt / sub
    while t < seconds + 1e-9:
        volts = profile(t)
        out.append((t, volts, p))
        for _ in range(sub):
            if abs(v) < 1e-9 and abs(volts) <= kS:
                v = 0.0
            else:
                s = math.copysign(1, v if abs(v) > 1e-9 else volts)
                v += (volts - kS * s - kV * v) / kA * h
            p += v * h
        t += dt
    return out


def _runs(kS, kV, kA, ramp, step):
    return [
        _plant(kS, kV, kA, lambda t: ramp * t, 8),
        _plant(kS, kV, kA, lambda t: -ramp * t, 8),
        _plant(kS, kV, kA, lambda t: step, 1.5),
        _plant(kS, kV, kA, lambda t: -step, 1.5),
    ]


@pytest.mark.parametrize(
    "kS,kV,kA,ramp,step",
    [
        (0.15, 0.25, 0.006, 0.5, 3.0),  # steering-like
        (0.12, 2.90, 0.40, 0.75, 5.0),  # drive-like
    ],
)
def test_fit_recovers_known_constants(kS, kV, kA, ramp, step):
    r = fit.fitFeedforward(_runs(kS, kV, kA, ramp, step))
    assert r.ok, r.describe()
    assert r.kS == pytest.approx(kS, rel=0.02)
    assert r.kV == pytest.approx(kV, rel=0.02)
    assert r.kA == pytest.approx(kA, rel=0.05)


def test_fit_with_encoder_quantization():
    """NEO built-in encoder: 42 counts per motor turn, about 1 mm of travel."""
    random.seed(0)
    quantum = 0.0426 / 42
    runs = [
        [(t, v, math.floor((p + off) / quantum) * quantum) for t, v, p in run]
        for run, off in zip(_runs(0.12, 2.9, 0.40, 0.75, 5.0), [random.random() * quantum] * 4)
    ]
    r = fit.fitFeedforward(runs)
    assert r.ok, r.describe()
    assert r.kV == pytest.approx(2.9, rel=0.03)
    assert r.kA == pytest.approx(0.40, rel=0.08)


def test_fit_rejects_backward_motor():
    """Position going the opposite way to the voltage must fail, not tune."""
    runs = [[(t, v, -p) for t, v, p in run] for run in _runs(0.12, 2.9, 0.4, 0.75, 5.0)]
    r = fit.fitFeedforward(runs)
    assert not r.ok
    assert "inversion" in r.reason


def test_fit_rejects_motor_that_never_moved():
    runs = [[(i * 0.02, 1.0, 0.0) for i in range(300)]]
    assert not fit.fitFeedforward(runs).ok


def test_delay_estimate():
    dt = 0.02
    true = [math.sin(0.15 * k) + 1.0 for k in range(200)]  # interval averages
    # measured at the end of interval k = true value 0.5 + 3 samples earlier
    meas = [math.sin(0.15 * (k + 0.5 - 3.5)) + 1.0 for k in range(200)]
    assert fit.estimateDelay(true, meas, dt) == pytest.approx(3.5 * dt, abs=0.25 * dt)


def test_gain_rules_are_sane():
    p = fit.steeringP(kV=0.25, kA=0.006, zeta=0.75)
    assert 0.1 < p < 2.0
    # more smoothing -> lower P
    assert fit.driveP(0.4, 0.06, 2.0) < fit.driveP(0.4, 0.06, 1.0)
    # longer lag -> lower P
    assert fit.driveP(0.4, 0.12) < fit.driveP(0.4, 0.06)


def test_step_response_metrics():
    ts = [i * 0.02 for i in range(60)]
    angles = [1.0 - math.exp(-t / 0.05) * math.cos(25 * t) for t in ts]
    over, settle = fit.stepResponse(ts, angles, 0.0, 1.0)
    assert 0.0 < over < 0.3
    assert 0.0 < settle < 1.0


# ---- whole autotune in the simulator -----------------------------------------------


def test_full_autotune_in_sim(control, robot, monkeypatch, tmp_path):
    import commands2
    from wpilib.simulation import DriverStationSim

    from easyswerve.sim import physics
    from easyswerve.tuning import gains

    monkeypatch.setenv(gains.ENV_OVERRIDE, str(tmp_path / "tuning.json"))
    monkeypatch.setattr(physics, "PRINT_PERIOD_S", 0)

    with control.run_robot():
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)
        DriverStationSim.setTest(True)
        control.step_timing(seconds=0.2, autonomous=False, enabled=True)

        tuner = findDrivetrain(robot).autotuner
        commands2.CommandScheduler.getInstance().schedule(tuner.fullCommand())
        control.step_timing(seconds=60, autonomous=False, enabled=True)

        assert not tuner.failed, "\n".join(tuner.lines)
        g = findDrivetrain(robot).gains
        assert g.source == "autotune"
        assert (tmp_path / "tuning.json").exists()

        # drive feedforward should match the simulated motor
        assert g.driveKV == pytest.approx(physics.SIM_DRIVE_KV, rel=0.05)
        assert g.driveKS == pytest.approx(physics.SIM_DRIVE_KS, abs=0.05)
        assert 0.05 < g.driveP <= 1.0
        assert 0.1 < g.steerP < 3.0
        # steering friction push: a fraction of the simulated steering friction
        from easyswerve.drivetrain.constants import AutotuneConstants

        assert g.steerKS == pytest.approx(
            AutotuneConstants.kSteerFrictionFraction * physics.SIM_STEER_KS, abs=0.03
        )

        DriverStationSim.setTest(False)
