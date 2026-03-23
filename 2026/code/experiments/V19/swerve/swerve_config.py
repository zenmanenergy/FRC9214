"""Swerve drive configuration constants"""
from ..CANID import CANID

# Manual offsets per wheel (added to zero calibration)
# front wheels are physically 180° rotated from rear wheels
MANUAL_OFFSETS = {
	"front_right": 180.0,
	"rear_right": 0.0,
	"rear_left": 90.0,
	"front_left": 270.0
}

# Motor and encoder pin assignments
WHEELS = {
	"front_right": {
		"drive_canid": CANID.SWERVE_FRONT_RIGHT_DRIVE,
		"turn_canid": CANID.SWERVE_FRONT_RIGHT_TURN,
		"encoder_dio": 0,
		"button": 4,  # Y
	},
	"rear_right": {
		"drive_canid": CANID.SWERVE_REAR_RIGHT_DRIVE,
		"turn_canid": CANID.SWERVE_REAR_RIGHT_TURN,
		"encoder_dio": 1,
		"button": 2,  # B
	},
	"rear_left": {
		"drive_canid": CANID.SWERVE_REAR_LEFT_DRIVE,
		"turn_canid": CANID.SWERVE_REAR_LEFT_TURN,
		"encoder_dio": 2,
		"button": 1,  # A
	},
	"front_left": {
		"drive_canid": CANID.SWERVE_FRONT_LEFT_DRIVE,
		"turn_canid": CANID.SWERVE_FRONT_LEFT_TURN,
		"encoder_dio": 3,
		"button": 3,  # X
	}
}

# Motor control scaling
MOTOR_SCALE_MANUAL = 0.05  # 5% max speed for manual control
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

# Rotation angles for 360° in-place spin
ROTATION_ANGLES = {
	"rear_left": 45,
	"rear_right": 135,
	"front_left": 315,
	"front_right": 225
}

# Alignment parameters
ALIGN_TOLERANCE = 5.0
ALIGN_TIMEOUT = 5.0

OFFSET_FILE = "/home/lvuser/encoder_offsets.json"
