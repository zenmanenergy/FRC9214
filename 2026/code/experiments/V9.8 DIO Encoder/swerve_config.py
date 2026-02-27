"""Swerve drive configuration constants"""

# Manual offsets per wheel (added to zero calibration)
MANUAL_OFFSETS = {
	"front_right": 0.0,
	"rear_right": 0.0,
	"rear_left": 0.0,
	"front_left": 0.0
}

# Motor and encoder pin assignments
WHEELS = {
	"front_right": {
		"drive_canid": 2,
		"turn_canid": 9,
		"encoder_dio": 0,
		"button": 4,  # Y
	},
	"rear_right": {
		"drive_canid": 4,
		"turn_canid": 3,
		"encoder_dio": 1,
		"button": 2,  # B
	},
	"rear_left": {
		"drive_canid": 6,
		"turn_canid": 5,
		"encoder_dio": 2,
		"button": 1,  # A
	},
	"front_left": {
		"drive_canid": 8,
		"turn_canid": 7,
		"encoder_dio": 3,
		"button": 3,  # X
	}
}

# Motor control scaling
MOTOR_SCALE_MANUAL = 0.05  # 5% max speed for manual control
MOTOR_SCALE_ALIGN = 0.3    # 30% max speed for alignment

# Alignment parameters
ALIGN_TOLERANCE = 2.0      # degrees
ALIGN_TIMEOUT = 5.0        # seconds
ALIGN_KP = 0.002           # proportional gain

# Offset file location
OFFSET_FILE = "/home/lvuser/encoder_offsets.json"
