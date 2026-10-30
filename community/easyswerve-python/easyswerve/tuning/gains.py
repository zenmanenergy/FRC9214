"""
The control gains the SPARK MAXes run with, and where autotune saves them.

At startup the drivetrain loads the saved gains. If autotune has never been
run, the defaults below (REV's template values) are used.

Units are the ones the SPARK MAX expects:
  driveP   duty cycle per (m/s) of speed error
  driveKS  volts, added in the direction of travel
  driveKV  volts per (m/s)
  steerP   duty cycle per radian of angle error
  steerKS  volts, friction push added in the direction of the steering error

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
    steerKS: float = 0.0

    # Bookkeeping, filled in by autotune
    source: str = "REV template defaults"
    tunedAt: str = ""
    notes: typing.Dict[str, typing.Any] = dataclasses.field(default_factory=dict)

    def summary(self) -> str:
        return (
            f"drive P={self.driveP:.4f} kS={self.driveKS:.3f} V kV={self.driveKV:.3f} V/(m/s) | "
            f"steer P={self.steerP:.3f} kS={self.steerKS:.3f} V ({self.source})"
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


def load() -> TunedGains:
    p = path()
    if not os.path.exists(p):
        return TunedGains()
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
