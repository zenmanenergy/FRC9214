"""Swerve drive configuration constants"""
import CANID

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
MOTOR_SCALE_ALIGN = 0.25    # 25% max speed for alignment (fast enough to track changing commands)
MOTOR_SCALE_TELEOP = 1.0   # 100% max speed for teleop drive

# Wheel positions relative to center (in arbitrary units, ratios matter)
# Positive X = right, Positive Y = forward
WHEEL_POSITIONS = {
	"front_right": {"x": 0.5, "y": 0.5},     # Right-Front
	"front_left": {"x": -0.5, "y": 0.5},    # Left-Front  
	"rear_right": {"x": 0.5, "y": -0.5},    # Right-Rear
	"rear_left": {"x": -0.5, "y": -0.5}     # Left-Rear
}

# Alignment parameters
ALIGN_TOLERANCE = 2.0      # degrees
ALIGN_TIMEOUT = 5.0        # seconds

# PID controller gains for wheel alignment
# KP: proportional gain (main response to error)
# KI: integral gain (eliminates steady-state error)
# KD: derivative gain (dampens oscillation) - VERY SMALL to avoid spikes
ALIGN_KP = 0.003           # proportional gain - reduced to prevent overshoot
ALIGN_KI = 0.005           # integral gain - increased for fine-tuning
ALIGN_KD = 0.0001          # derivative gain - nearly zero, was causing oscillation spikes

# Offset file location
OFFSET_FILE = "/home/lvuser/encoder_offsets.json"
