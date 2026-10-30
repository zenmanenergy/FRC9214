"""
Tests for the vision accept/reject rules and trust sizing (easyswerve_vision/filters.py).
Plain Python: no robot or camera needed. Copy into your robot project's tests/
folder and run: python -m robotpy test
"""

import dataclasses

import pytest

from easyswerve.vision import config as C
from easyswerve.vision import filters


def est(**kw):
    base = dict(
        x=3.0, y=4.0, z=0.0, tagCount=2, avgDistanceM=2.0, ambiguity=0.0,
        spinDegPerS=0.0, fieldLengthM=16.54, fieldWidthM=8.07,
    )
    base.update(kw)
    return filters.Estimate(**base)


def test_good_multi_tag_accepted():
    v = filters.judge(est())
    assert v.accepted and v.reason == "ok"
    assert v.stdDevs[2] < 1.0  # multi-tag may correct heading


def test_single_tag_never_corrects_heading():
    v = filters.judge(est(tagCount=1, ambiguity=0.05))
    assert v.accepted
    assert v.stdDevs[2] >= 1e5


@pytest.mark.parametrize(
    "change,reason",
    [
        (dict(tagCount=0), "no tags"),
        (dict(spinDegPerS=500.0), "spinning too fast"),
        (dict(z=1.2), "pose not on the floor"),
        (dict(x=-2.0), "off the field"),
        (dict(y=20.0), "off the field"),
        (dict(tagCount=1, ambiguity=0.5), "single tag ambiguous"),
        (dict(tagCount=1, ambiguity=0.05, avgDistanceM=5.0), "single tag too far"),
        (dict(tagCount=3, avgDistanceM=8.0), "tags too far"),
    ],
)
def test_rejections(change, reason):
    v = filters.judge(est(**change))
    assert not v.accepted
    assert v.reason == reason


def test_trust_drops_with_distance_and_rises_with_tags():
    near = filters.stdDevs(2, 1.0)[0]
    far = filters.stdDevs(2, 3.0)[0]
    more = filters.stdDevs(4, 3.0)[0]
    assert far > near
    assert more < far


def test_trust_has_a_floor():
    assert filters.stdDevs(4, 0.1)[0] == pytest.approx(C.MIN_XY_STD_M)


def test_unmeasured_camera_is_watch_only_on_robot():
    cam = C.CameraConfig("front", 0.3, 0.0, 0.25, measured=False)
    usable, why = filters.cameraUsable(cam, isSimulation=False)
    assert not usable and "watch only" in why


def test_unmeasured_camera_fuses_in_sim_when_allowed(monkeypatch):
    monkeypatch.setattr(C, "SIM_USE_UNMEASURED_CAMERAS", True)
    cam = C.CameraConfig("front", 0.3, 0.0, 0.25, measured=False)
    assert filters.cameraUsable(cam, isSimulation=True)[0]


def test_measured_camera_fuses():
    cam = C.CameraConfig("front", 0.3, 0.0, 0.25, measured=True)
    assert filters.cameraUsable(cam, isSimulation=False) == (True, "fusing")


def test_disabled_camera_ignored():
    cam = C.CameraConfig("front", 0.3, 0.0, 0.25, measured=True, enabled=False)
    assert not filters.cameraUsable(cam, isSimulation=False)[0]


def test_placeholder_camera_ships_unmeasured():
    """The shipped config must never fuse on the real robot until someone
    measures the camera."""
    assert all(not c.measured for c in C.CAMERAS)
