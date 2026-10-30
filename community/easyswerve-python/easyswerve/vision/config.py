"""
Vision settings (advanced). Your cameras and the on/off switch are in
robot_settings.py (CAMERAS, VISION_ENABLED); this file holds the filter and
trust settings, which rarely change. See docs/VISION.md.

While a camera's "measured" is False, the code still connects to it and shows
what it sees on the dashboard, but its position estimates are NOT used to move
the robot's position (a wrong camera position would drag the robot's position
the wrong way).

All lengths in meters, angles in degrees.
"""

import dataclasses
import typing


@dataclasses.dataclass
class CameraConfig:
    # Camera name exactly as shown in the PhotonVision web page (Cameras tab)
    name: str

    # Where the camera lens is, measured from the robot's center on the floor
    forwardM: float  # + toward the robot's front
    leftM: float  # + toward the robot's left side
    upM: float  # height of the lens above the floor

    # Which way it points
    yawDeg: float = 0.0  # 0 = straight ahead, 90 = left, -90 = right, 180 = back
    tiltUpDeg: float = 0.0  # + = tilted up from level, - = tilted down
    rollDeg: float = 0.0  # usually 0 (camera mounted upright)

    # Set True once the numbers above are measured on the real robot
    measured: bool = False

    # Set False to ignore this camera completely
    enabled: bool = True


# ---- your cameras: from robot_settings.py ---------------------------------------
import robot_settings as _RS  # noqa: E402

CAMERAS: typing.List[CameraConfig] = [
    CameraConfig(
        name=c.name,
        forwardM=c.forward_m,
        leftM=c.left_m,
        upM=c.up_m,
        yawDeg=c.yaw_deg,
        tiltUpDeg=c.tilt_up_deg,
        rollDeg=c.roll_deg,
        measured=c.measured,
        enabled=c.enabled,
    )
    for c in getattr(_RS, "CAMERAS", [])
]

# Master switch. False = never use cameras (pure wheel + gyro tracking).
VISION_ENABLED = bool(getattr(_RS, "VISION_ENABLED", False))

# In the simulator the camera IS where this file says, so placeholder values
# are self-consistent there. True lets you try fusion in the sim before you
# measure anything. It never affects the real robot.
SIM_USE_UNMEASURED_CAMERAS = True

# Field AprilTag layout. None = the current season's field from robotpy_apriltag
# (AprilTagField.kDefaultField). To use another season's field, give the name of
# a robotpy_apriltag.AprilTagField member, as listed in RobotPy's docs.
FIELD_LAYOUT: typing.Optional[str] = None

# ---- when to ignore a camera estimate ----------------------------------------------
# A single tag can be read two ways (ambiguity); above this, ignore it (0..1)
MAX_SINGLE_TAG_AMBIGUITY = 0.2
# Ignore single-tag estimates from farther than this (multi-tag allowed farther)
MAX_SINGLE_TAG_DISTANCE_M = 4.0
MAX_MULTI_TAG_DISTANCE_M = 6.0
# Ignore estimates while the robot spins faster than this (motion blur)
MAX_SPIN_DEG_PER_S = 360.0
# Ignore estimates this far outside the field, or this far above/below the floor
FIELD_MARGIN_M = 0.5
MAX_HEIGHT_ERROR_M = 0.5

# ---- how much to trust an accepted estimate ----------------------------------------
# Standard deviation grows with distance squared and shrinks with more tags:
#   xy  = XY_COEFFICIENT * distance^2 / tags      (meters)
#   heading = HEADING_COEFFICIENT * distance^2 / tags   (radians)
# Single tags never correct heading (the navX is far better at that).
XY_COEFFICIENT = 0.02
HEADING_COEFFICIENT = 0.06
MIN_XY_STD_M = 0.02
SINGLE_TAG_MULTIPLIER = 2.0  # single tags trusted half as much

# ---- position at the start of a match --------------------------------------------
# While disabled, if a camera sees at least this many tags, set the robot's
# position straight to the camera's (no blending). Puts the robot in the right
# spot on the field before auto. 0 = never.
SEED_WHILE_DISABLED_MIN_TAGS = 2

# ---- simulator ------------------------------------------------------------------------
# Simulated camera model (photonlibpy SimCameraProperties): one of
# "OV9281_640_480", "OV9281_800_600", "OV9281_1280_720", "PERFECT_90DEG", ...
SIM_CAMERA_MODEL = "OV9281_800_600"
SIM_CAMERA_FPS = 30.0
SIM_CAMERA_LATENCY_MS = 35.0
