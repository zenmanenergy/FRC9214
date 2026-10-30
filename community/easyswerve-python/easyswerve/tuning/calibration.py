"""
Steering encoder zero offsets for encoders wired to the roboRIO.

The SPARK's own zero-offset utility (REV Hardware Client) only works for
encoders on the SPARK data port. With roboRIO wiring, the zero is stored here
instead: put the hex keys in, press the dashboard button, and the current
readings become the zeros.

Kept separate from the tuning file, so resetting autotune never loses the
encoder zeros.
"""

import json
import os
import time
import typing

import wpilib

ROBORIO_PATH = "/home/lvuser/easyswerve-calibration.json"
SIM_FILENAME = "easyswerve-calibration.json"
ENV_OVERRIDE = "EASYSWERVE_CALIBRATION_FILE"


def path() -> str:
    override = os.environ.get(ENV_OVERRIDE)
    if override:
        return override
    if wpilib.RobotBase.isReal():
        return ROBORIO_PATH
    # the robot project folder (easyswerve/tuning/ is two levels down)
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(here, SIM_FILENAME)


def loadOffsets() -> typing.Optional[typing.Dict[str, float]]:
    """Saved offsets by module name (radians), or None if never zeroed."""
    p = path()
    if not os.path.exists(p):
        return None
    try:
        with open(p) as f:
            data = json.load(f)
        return {k: float(v) for k, v in data["offsetsRad"].items()}
    except Exception as e:  # a corrupt file must never stop the robot
        wpilib.reportError(f"Could not read steering zeros from {p} ({e})", False)
        return None


def saveOffsets(offsets: typing.Dict[str, float]) -> str:
    p = path()
    tmp = p + ".tmp"
    with open(tmp, "w") as f:
        json.dump(
            {"offsetsRad": offsets, "zeroedAt": time.strftime("%Y-%m-%d %H:%M:%S")},
            f,
            indent=2,
        )
    os.replace(tmp, p)
    return p
