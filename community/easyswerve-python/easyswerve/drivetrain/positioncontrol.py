"""
PID position control: drive to a spot on the field and face a direction.

Three WPILib PIDControllers (x, y and heading) compare where the robot is with
where it should be, every loop, and ask for the field speeds that close the
gap. The gains are tuned by the position autotune (tuning/positiontune.py)
and stored with the other gains (posP/posI/posD, headP/headI/headD).

The speed it asks for is also limited: never faster than maxMps, and it
speeds up at most accelMps2 (so the wheels don't slip, which would make the
encoders wrong). Slowing down isn't limited: the PID does that near the goal.
"""

import math

from wpimath.controller import PIDController
from wpimath.geometry import Pose2d

LOOP_S = 0.02


def positionPID(gains) -> PIDController:
    pid = PIDController(gains.posP, gains.posI, gains.posD)
    pid.setIZone(gains.posIZoneM)  # I only works close to the target
    return pid


def headingPID(gains) -> PIDController:
    pid = PIDController(gains.headP, gains.headI, gains.headD)
    pid.enableContinuousInput(-math.pi, math.pi)  # 179 and -179 degrees are 2 apart
    pid.setIZone(math.radians(gains.headIZoneDeg))
    return pid


class PositionController:
    def __init__(self, gains, maxMps: float, maxRadPerS: float, accelMps2: float = 3.0) -> None:
        self.x = positionPID(gains)
        self.y = positionPID(gains)
        self.heading = headingPID(gains)
        self.maxMps = maxMps
        self.maxRadPerS = maxRadPerS
        self.accelMps2 = accelMps2
        self.lastMps = 0.0

    def reset(self) -> None:
        self.x.reset()
        self.y.reset()
        self.heading.reset()
        self.lastMps = 0.0

    def calculate(self, pose: Pose2d, target: Pose2d):
        """Field-relative (vx m/s, vy m/s, omega rad/s) toward the target."""
        vx = self.x.calculate(pose.X(), target.X())
        vy = self.y.calculate(pose.Y(), target.Y())
        speed = math.hypot(vx, vy)
        allowed = min(self.maxMps, self.lastMps + self.accelMps2 * LOOP_S)  # smooth speed-up
        if speed > allowed and speed > 0:
            vx, vy = vx * allowed / speed, vy * allowed / speed
            speed = allowed
        self.lastMps = speed
        omega = self.heading.calculate(pose.rotation().radians(), target.rotation().radians())
        omega = max(-self.maxRadPerS, min(self.maxRadPerS, omega))
        return vx, vy, omega
