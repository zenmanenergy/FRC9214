"""Small record types used in robot_settings.py."""

import dataclasses


@dataclasses.dataclass(frozen=True)
class Module:
    """One swerve module's wiring (see robot_settings.py)."""

    drive_can: int
    turn_can: int
    encoder_dio: int = -1
    drive_motor_bottom: bool = True
    turn_motor_bottom: bool = True
    encoder_reversed: bool = False


@dataclasses.dataclass(frozen=True)
class Camera:
    """One PhotonVision camera (see robot_settings.py and docs/VISION.md).

    Measured from the robot's center on the floor: forward_m (+ toward the
    front), left_m (+ toward the left), up_m (lens height). yaw_deg: 0 ahead,
    90 left, -90 right. tilt_up_deg: + tilted up.
    """

    name: str
    forward_m: float
    left_m: float
    up_m: float
    yaw_deg: float = 0.0
    tilt_up_deg: float = 0.0
    roll_deg: float = 0.0
    measured: bool = False
    enabled: bool = True
