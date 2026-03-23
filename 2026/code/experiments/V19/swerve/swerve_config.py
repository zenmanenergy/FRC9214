"""Swerve drive configuration constants"""
from ..CANID import CANID

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
		"manual_offset": 180.0,
		"rotation_angle": 225,
	},
	"rear_right": {
		"drive_canid": CANID.SWERVE_REAR_RIGHT_DRIVE,
		"turn_canid": CANID.SWERVE_REAR_RIGHT_TURN,
		"encoder_dio": 1,
		"button": 2,  # B
		"manual_offset": 0.0,
		"rotation_angle": 135,
	},
	"rear_left": {
		"drive_canid": CANID.SWERVE_REAR_LEFT_DRIVE,
		"turn_canid": CANID.SWERVE_REAR_LEFT_TURN,
		"encoder_dio": 2,
		"button": 1,  # A
		"manual_offset": 90.0,
		"rotation_angle": 45,
	},
	"front_left": {
		"drive_canid": CANID.SWERVE_FRONT_LEFT_DRIVE,
		"turn_canid": CANID.SWERVE_FRONT_LEFT_TURN,
		"encoder_dio": 3,
		"button": 3,  # X
		"manual_offset": 270.0,
		"rotation_angle": 315,
	}
}

# Motor control scaling
MOTOR_SCALE_ALIGN = 0.3    # 15% max speed for alignment - reduced for better damping
MOTOR_SCALE_TELEOP = 0.75   # 100% max speed for teleop drive

# Wheel positions relative to center (in arbitrary units, ratios matter)
# Positive X = right, Positive Y = forward
WHEEL_POSITIONS = {
	"front_right": {"x": 0.5, "y": 0.5},     # Right-Front
	"front_left": {"x": -0.5, "y": 0.5},    # Left-Front  
	"rear_right": {"x": 0.5, "y": -0.5},    # Right-Rear
	"rear_left": {"x": -0.5, "y": -0.5}     # Left-Rear
}

# Alignment parameters
ALIGN_TOLERANCE = 5.0
ALIGN_TIMEOUT = 5.0

OFFSET_FILE = "/home/lvuser/encoder_offsets.json"
