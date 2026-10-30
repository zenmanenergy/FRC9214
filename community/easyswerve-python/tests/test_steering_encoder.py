"""
Tests for steering encoders wired to the roboRIO (subsystems/steeringencoder.py).

The simulator always keeps the two steering sensors in perfect agreement, so
these tests drive RioSteeringEncoder directly with stand-in sensors to create
the situations that happen on a real robot: startup, drift, a moving wheel, an
unplugged encoder and a stuck encoder.

    python -m robotpy test
"""

import math

import pytest

from easyswerve.drivetrain import steeringencoder as se

TAU = 2 * math.pi


class FakeNeoSensor:
    """The turning SPARK's built-in sensor, in module radians."""

    def __init__(self, position=0.0):
        self.position = position
        self.velocity = 0.0
        self.sets = 0

    def getPosition(self):
        return self.position

    def getVelocity(self):
        return self.velocity

    def setPosition(self, p):
        self.position = p
        self.sets += 1


class FakeSpark:
    def __init__(self, sensor):
        self.sensor = sensor

    def getEncoder(self):
        return self.sensor


class FakeDutyCycleEncoder:
    """The Through Bore Encoder's absolute output on a DIO port."""

    def __init__(self, channel, fullRange, expectedZero):
        self.channel = channel
        self.value = 0.0
        self.connected = True

    def setDutyCycleRange(self, low, high):
        pass

    def setAssumedFrequency(self, f):
        pass

    def isConnected(self):
        return self.connected

    def get(self):
        return self.value


@pytest.fixture
def enc(monkeypatch):
    monkeypatch.setattr(se.wpilib, "DutyCycleEncoder", FakeDutyCycleEncoder)
    warnings = []
    monkeypatch.setattr(se.wpilib, "reportWarning", lambda msg, *a: warnings.append(msg))
    neo = FakeNeoSensor(0.0)
    e = se.RioSteeringEncoder(FakeSpark(neo), dioChannel=3, offset=0.2, name="RR")
    e.neo = neo
    e.warnings = warnings
    return e


def _angle_err_deg(a, b):
    return abs(math.degrees((a - b + math.pi) % TAU - math.pi))


def test_startup_sets_neo_sensor_to_true_angle(enc):
    enc.absolute.value = 1.2  # raw; true angle = 1.2 - 0.2 offset = 1.0
    assert not enc.synced
    enc.sync(enabled=False, now=0.0)
    assert enc.synced
    assert _angle_err_deg(enc.getPosition(), 1.0) < 1e-6


def test_drift_corrected_when_wheel_still(enc):
    enc.absolute.value = 1.2
    enc.sync(False, 0.0)
    enc.neo.position += math.radians(2.0)  # 2 degrees of drift (backlash)
    enc.sync(True, 2.0)
    assert _angle_err_deg(enc.getPosition(), 1.0) < 0.01
    assert enc.resyncCount == 1


def test_small_disagreement_left_alone_while_enabled(enc):
    enc.absolute.value = 1.2
    enc.sync(False, 0.0)
    enc.neo.position += math.radians(0.5)  # under the 1 degree threshold
    enc.sync(True, 2.0)
    assert enc.resyncCount == 0


def test_no_correction_while_wheel_moving(enc):
    enc.absolute.value = 1.2
    enc.sync(False, 0.0)
    enc.neo.position += math.radians(5.0)
    enc.neo.velocity = 3.0  # rad/s: steering in progress
    enc.sync(True, 2.0)
    assert enc.resyncCount == 0


def test_disabled_resync_is_rate_limited(enc):
    enc.absolute.value = 1.2
    enc.sync(False, 0.0)
    enc.neo.position += math.radians(3.0)
    enc.sync(False, 0.1)  # too soon
    assert enc.resyncCount == 0
    enc.sync(False, 0.6)
    assert enc.resyncCount == 1


def test_unplugged_encoder_never_syncs_and_warns(enc):
    enc.absolute.connected = False
    enc.sync(False, 0.0)
    assert not enc.synced
    assert enc.absoluteAngle() is None
    assert any("no signal" in w for w in enc.warnings)


def test_stuck_encoder_is_ignored(enc):
    enc.absolute.value = 1.2
    enc.sync(False, 0.0)
    # wheel turns 90 degrees in small steps, absolute value frozen
    for i in range(1, 19):
        enc.neo.position += math.radians(5.0)
        enc.sync(True, 0.02 * i)
    assert enc.stuck
    assert any("reads the same value" in w for w in enc.warnings)
    # it must NOT have dragged the NEO sensor back to the frozen value
    enc.neo.velocity = 0.0
    before = enc.neo.position
    enc.sync(True, 5.0)
    assert enc.neo.position == before
    # encoder starts changing again: trusted again
    enc.absolute.value = 1.2 + math.radians(90.0)
    enc.sync(True, 6.0)
    assert not enc.stuck


def test_wraparound_takes_short_way(enc):
    enc.absolute.value = (0.2 + math.radians(359.0)) % TAU  # true angle 359 deg
    enc.neo.position = math.radians(721.0)  # 1 deg, two turns up
    enc.sync(False, 0.0)
    # corrected by -2 degrees, not by +358
    assert abs(enc.neo.position - math.radians(719.0)) < 1e-9


def test_set_offset_resyncs(enc):
    enc.absolute.value = 1.2
    enc.sync(False, 0.0)
    enc.setOffset(1.2)  # zeroing with the hex key in: current reading is zero
    assert not enc.synced
    enc.sync(False, 0.1)
    assert _angle_err_deg(enc.getPosition(), 0.0) < 1e-6


def test_bad_encoder_version_rejected(monkeypatch):
    monkeypatch.setattr(se.wpilib, "DutyCycleEncoder", FakeDutyCycleEncoder)
    monkeypatch.setattr(se.ModuleConstants, "kThroughBoreEncoderVersion", 3)
    with pytest.raises(ValueError):
        se.RioSteeringEncoder(FakeSpark(FakeNeoSensor()), 0, 0.0, "FL")
