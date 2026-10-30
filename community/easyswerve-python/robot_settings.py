"""
ROBOT SETTINGS

Everything that is specific to YOUR robot is in this file. Measure, check
the wiring, and fill in the values. Lengths are in meters.

New to this? examples/00_00_robotsettings/README.md explains every line
and how to find each value on your robot.

(Advanced settings that rarely change live in
easyswerve/drivetrain/constants.py. You shouldn't need them.)
"""

from easyswerve.settings_types import Camera, Module

# ---- Swerve modules ------------------------------------------------------------
# One line per corner. Look at the robot from above with its front pointing up.
#
#   drive_can          CAN ID of the SPARK MAX that spins the wheel
#   turn_can           CAN ID of the SPARK MAX that steers the wheel
#   encoder_dio        roboRIO DIO port the steering encoder is plugged into
#   drive_motor_bottom True if the drive motor is mounted under the module
#   turn_motor_bottom  True if the steering motor is mounted under the module
#   encoder_reversed   True if the steering encoder counts backwards (leave it
#                      out until examples/00_03_steeronebyone says to add it)
#
# Not sure? Leave them as they are: the 00_ examples check every one.

FRONT_LEFT = Module(drive_can=9, turn_can=8, encoder_dio=0, drive_motor_bottom=True, turn_motor_bottom=True)
FRONT_RIGHT = Module(drive_can=11, turn_can=10, encoder_dio=1, drive_motor_bottom=True, turn_motor_bottom=True)
BACK_LEFT = Module(drive_can=15, turn_can=14, encoder_dio=2, drive_motor_bottom=True, turn_motor_bottom=True)
BACK_RIGHT = Module(drive_can=13, turn_can=12, encoder_dio=3, drive_motor_bottom=True, turn_motor_bottom=True)

# ---- Steering encoders -----------------------------------------------------------
# Where the steering encoders are plugged in:
#   "rio"   = roboRIO DIO ports (the encoder_dio numbers above)
#   "spark" = the steering SPARK MAX's data port (encoder_dio is then ignored)
STEERING_ENCODERS = "rio"

# Through Bore Encoder model: 1 = original (REV-11-1271), 2 = V2 (REV-11-3174)
ENCODER_VERSION = 1

# Backup of the steering zeros, in degrees. The Calibrate button saves the
# zeros on the roboRIO; afterwards the console prints a line to paste here.
# Only used if the saved zeros are missing (for example after the roboRIO is
# reimaged). None = no backup yet.
STEERING_ZEROS_DEG = None

# ---- Robot size ---------------------------------------------------------------------
# Measured between the CENTERS of the wheels
TRACK_WIDTH_M = 0.6858  # left wheels to right wheels
WHEEL_BASE_M = 0.6858  # front wheels to back wheels

# ---- Wheels -------------------------------------------------------------------------
# Wheel diameter. New EasySwerve wheels are 0.1016 m; treads wear smaller.
# examples/00_10_wheeldiameter measures the right value for your robot.
WHEEL_DIAMETER_M = 0.1016

# ---- Driving -----------------------------------------------------------------------
MAX_SPEED_MPS = 4.5  # full stick speed (the wheels can do about 4.8)
MAX_TURN_DEG_PER_S = 360  # full stick spin speed
CONTROLLER_PORT = 0  # Driver Station USB slot of the driver's controller

# ---- Gyro ----------------------------------------------------------------------------
# Where the navX2 is plugged in: "mxp" (on top of the roboRIO) or "usb"
GYRO = "mxp"

# ---- Cameras (PhotonVision) ---------------------------------------------------------
# Leave VISION_ENABLED = False until the Orange Pi and cameras are set up.
# docs/VISION.md explains how to measure the camera numbers.
VISION_ENABLED = False
CAMERAS = [
    # PLACEHOLDER values: measure, then set measured=True
    Camera(name="front", forward_m=0.30, left_m=0.0, up_m=0.25, yaw_deg=0, tilt_up_deg=20, measured=False),
    Camera(name="back", forward_m=-0.30, left_m=0.0, up_m=0.25, yaw_deg=180, tilt_up_deg=20, measured=False),
]
