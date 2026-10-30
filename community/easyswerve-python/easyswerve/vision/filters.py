"""
Decides whether to use a camera position estimate and how much to trust it.

Plain numbers in, plain numbers out (no robot libraries), so it is easy to
test and to reason about.
"""

import dataclasses
import math
import typing

from . import config as C


@dataclasses.dataclass
class Estimate:
    """What one camera frame says, reduced to the numbers the filter needs."""

    x: float  # robot position on the field, m
    y: float
    z: float  # height of the estimated robot pose (should be about 0)
    tagCount: int
    avgDistanceM: float  # average camera-to-tag distance
    ambiguity: float  # single-tag ambiguity (0..1); ignored for multi-tag
    spinDegPerS: float  # robot rotation rate at the time
    fieldLengthM: float
    fieldWidthM: float


@dataclasses.dataclass
class Verdict:
    accepted: bool
    reason: str  # why rejected, or "ok"
    stdDevs: typing.Tuple[float, float, float] = (0.0, 0.0, 0.0)


def judge(e: Estimate) -> Verdict:
    """Accept or reject an estimate, and size its trust if accepted."""
    if e.tagCount <= 0:
        return Verdict(False, "no tags")
    if abs(e.spinDegPerS) > C.MAX_SPIN_DEG_PER_S:
        return Verdict(False, "spinning too fast")
    if abs(e.z) > C.MAX_HEIGHT_ERROR_M:
        return Verdict(False, "pose not on the floor")
    m = C.FIELD_MARGIN_M
    if not (-m <= e.x <= e.fieldLengthM + m and -m <= e.y <= e.fieldWidthM + m):
        return Verdict(False, "off the field")

    single = e.tagCount == 1
    if single:
        if e.ambiguity > C.MAX_SINGLE_TAG_AMBIGUITY:
            return Verdict(False, "single tag ambiguous")
        if e.avgDistanceM > C.MAX_SINGLE_TAG_DISTANCE_M:
            return Verdict(False, "single tag too far")
    elif e.avgDistanceM > C.MAX_MULTI_TAG_DISTANCE_M:
        return Verdict(False, "tags too far")

    return Verdict(True, "ok", stdDevs(e.tagCount, e.avgDistanceM))


def stdDevs(tagCount: int, avgDistanceM: float) -> typing.Tuple[float, float, float]:
    """(x m, y m, heading rad): bigger = trusted less."""
    d2 = avgDistanceM * avgDistanceM
    n = max(1, tagCount)
    xy = max(C.MIN_XY_STD_M, C.XY_COEFFICIENT * d2 / n)
    if n == 1:
        xy *= C.SINGLE_TAG_MULTIPLIER
        heading = 1e6  # never correct heading from one tag
    else:
        heading = C.HEADING_COEFFICIENT * d2 / n
    return (xy, xy, heading)


def cameraUsable(cam: "C.CameraConfig", isSimulation: bool) -> typing.Tuple[bool, str]:
    """Whether a camera's estimates may move the robot's position."""
    if not cam.enabled:
        return False, "disabled in config"
    if cam.measured:
        return True, "fusing"
    if isSimulation and C.SIM_USE_UNMEASURED_CAMERAS:
        return True, "fusing (sim, placeholder position)"
    return False, "watch only: set measured=True after measuring"
