"""Swerve drive configuration constants"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from CANID import CANID

# Motor and encoder pin assignments with manual offsets and rotation angles
# Manual offsets represent physical mounting rotation (added to zero calibration)
# Front wheels are physically 180° rotated from rear wheels
# Rotation angles are used for 360° in-place spin
WHEELS = {
	"front_right": {
		"drive_canid": CANID.SWERVE_FRONT_RIGHT_DRIVE,
		"turn_canid": CANID.SWERVE_FRONT_RIGHT_TURN,
		"encoder_dio": 0,
		"button": 4,  # Y
		"manual_offset": 0.0,
		"rotation_angle": 225,
		"position": {"x": 0.5, "y": 0.5},
	},
	"rear_right": {
		"drive_canid": CANID.SWERVE_REAR_RIGHT_DRIVE,
		"turn_canid": CANID.SWERVE_REAR_RIGHT_TURN,
		"encoder_dio": 1,
		"button": 2,  # B
		"manual_offset": 0.0,
		"rotation_angle": 135,
		"position": {"x": 0.5, "y": -0.5},
	},
	"rear_left": {
		"drive_canid": CANID.SWERVE_REAR_LEFT_DRIVE,
		"turn_canid": CANID.SWERVE_REAR_LEFT_TURN,
		"encoder_dio": 2,
		"button": 1,  # A
		"manual_offset": 0.0,
		"rotation_angle": 45,
		"position": {"x": -0.5, "y": -0.5},
	},
	"front_left": {
		"drive_canid": CANID.SWERVE_FRONT_LEFT_DRIVE,
		"turn_canid": CANID.SWERVE_FRONT_LEFT_TURN,
		"encoder_dio": 3,
		"button": 3,  # X
		"manual_offset": 0.0,
		"rotation_angle": 315,
		"position": {"x": -0.5, "y": 0.5},
	}
}

# Motor control scaling
MOTOR_SCALE_ALIGN = 0.3    # 15% max speed for alignment - reduced for better damping
MOTOR_SCALE_TELEOP = 0.75   # 100% max speed for teleop drive

# Alignment parameters
ALIGN_TOLERANCE = 5.0
ALIGN_TIMEOUT = 5.0

# Odometry trust decay (lower trust over time without external verification)
# Decay per meter of distance traveled
ODOMETRY_POSITION_DECAY_PER_METER = 0.01
ODOMETRY_HEADING_DECAY_PER_METER = 0.02

# Decay per second of elapsed time (independent of movement)
ODOMETRY_POSITION_DECAY_PER_SECOND = 0.005
ODOMETRY_HEADING_DECAY_PER_SECOND = 0.01

# Never let trust fall below this value when decaying
ODOMETRY_MIN_TRUST = 0.1

OFFSET_FILE = "/home/lvuser/encoder_offsets.json"
