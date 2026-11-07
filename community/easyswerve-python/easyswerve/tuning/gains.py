"""
The control gains the SPARK MAXes run with, and where autotune saves them.

At startup the drivetrain loads the saved gains. If autotune has never been
run, the defaults below (REV's template values) are used.

Units are the ones the SPARK MAX expects:
  driveP   duty cycle per (m/s) of speed error
  driveKS  volts, added in the direction of travel
  driveKV  volts per (m/s)
  steerP   duty cycle per radian of angle error
  steerI   duty cycle per (radian x second) of angle error (Ziegler-Nichols)
  steerD   duty cycle per (radian per second) of angle change (Ziegler-Nichols)
  steerIZoneDeg  the I term only works within this many degrees of the target
  steerKS  volts, friction push added in the direction of the steering error

Position and heading (driving to a spot: driveDistance, driveTo, goToPath,
PathPlanner autos; turning: turnTo, heading hold), from the position autotune:
  posP     m/s of correction per meter of position error
  posI     m/s per (meter x second) of error, only within posIZoneM of the target
  posD     m/s per (m/s) of the error changing
  headP    rad/s of turn per radian of heading error (the same per degree)
  headI    rad/s per (radian x second), only within headIZoneDeg of the target
  headD    rad/s per (rad/s) of the error changing

steerI and steerD are in ordinary per-second units, as textbooks and
Ziegler-Nichols give them. configs.py converts them for the SPARK, whose loop
runs every 1 ms.

Duty cycle is a fraction of ModuleConstants.kNominalVoltage, because the SPARKs
run with voltage compensation.
"""

import dataclasses
import json
import logging
import os
import time
import typing

import wpilib

import robot_settings as RS

from ..drivetrain.constants import ModuleConstants

logger = logging.getLogger("easyswerve.tuning")

# On the roboRIO this lives outside the deploy folder, so redeploying code
# never wipes your tuning. In the simulator it lives in the project folder.
ROBORIO_PATH = "/home/lvuser/easyswerve-tuning.json"
SIM_FILENAME = "easyswerve-tuning.json"
ENV_OVERRIDE = "EASYSWERVE_TUNING_FILE"


@dataclasses.dataclass
class TunedGains:
    driveP: float = 0.04
    driveKS: float = 0.0
    driveKV: float = 12.0 / ModuleConstants.kDriveWheelFreeSpeedRps
    steerP: float = 1.0
    steerI: float = 0.0
    steerD: float = 0.0
    steerIZoneDeg: float = 0.0
    steerKS: float = 0.0
    posP: float = 5.0
    posI: float = 0.0
    posD: float = 0.0
    posIZoneM: float = 0.03
    headP: float = 5.0
    headI: float = 0.0
    headD: float = 0.0
    headIZoneDeg: float = 3.0

    # Bookkeeping, filled in by autotune
    source: str = "REV template defaults"
    tunedAt: str = ""
    notes: typing.Dict[str, typing.Any] = dataclasses.field(default_factory=dict)

    def summary(self) -> str:
        return (
            f"drive P={self.driveP:.4f} kS={self.driveKS:.3f} V kV={self.driveKV:.3f} V/(m/s) | "
            f"steer P={self.steerP:.3f}"
            + (f" I={self.steerI:.3f} D={self.steerD:.5f}" if self.steerI or self.steerD else "")
            + f" kS={self.steerKS:.3f} V"
            + f" | position PID {self.posP:.2f}/{self.posI:.2f}/{self.posD:.3f}"
            + f" heading PID {self.headP:.2f}/{self.headI:.2f}/{self.headD:.3f} ({self.source})"
        )


def path() -> str:
    override = os.environ.get(ENV_OVERRIDE)
    if override:
        return override
    if wpilib.RobotBase.isReal():
        return ROBORIO_PATH
    # the robot project folder (easyswerve/tuning/ is two levels down)
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(here, SIM_FILENAME)


GAIN_FIELDS = [f.name for f in dataclasses.fields(TunedGains) if f.type in (float, "float")]


def backupLine(gains: TunedGains) -> str:
    """A line to paste into robot_settings.py, so the tuning survives a
    roboRIO that is reimaged or swapped (the saved file lives on the roboRIO)."""
    values = ", ".join(f'"{n}": {float(f"{getattr(gains, n):.5g}")}' for n in GAIN_FIELDS)
    return "TUNED_GAINS = {" + values + "}"


def _backup() -> typing.Optional[TunedGains]:
    """The TUNED_GAINS backup from robot_settings.py, or None."""
    data = getattr(RS, "TUNED_GAINS", None)
    if not data:
        return None
    try:
        known = set(GAIN_FIELDS)
        g = TunedGains(**{k: float(v) for k, v in data.items() if k in known})
        g.source = "backup from robot_settings.py (saved tuning missing)"
        return g
    except Exception as e:
        logger.error("TUNED_GAINS in robot_settings.py could not be read (%s); using defaults", e)
        return None


def load() -> TunedGains:
    p = path()
    if not os.path.exists(p):
        return _backup() or TunedGains()
    try:
        with open(p) as f:
            data = json.load(f)
        known = {f.name for f in dataclasses.fields(TunedGains)}
        gains = TunedGains(**{k: v for k, v in data.items() if k in known})
        logger.info("Loaded tuned gains from %s: %s", p, gains.summary())
        return gains
    except Exception as e:  # a corrupt file must never stop the robot from driving
        logger.error("Could not read %s (%s); using defaults", p, e)
        return TunedGains()


def save(gains: TunedGains) -> str:
    gains.tunedAt = time.strftime("%Y-%m-%d %H:%M:%S")
    p = path()
    tmp = p + ".tmp"
    with open(tmp, "w") as f:
        json.dump(dataclasses.asdict(gains), f, indent=2)
    os.replace(tmp, p)
    return p


def reset() -> str:
    """Delete the saved tuning so the defaults are used from the next start."""
    p = path()
    if os.path.exists(p):
        os.remove(p)
    return p
