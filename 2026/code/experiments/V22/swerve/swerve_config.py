"""Swerve drive configuration constants.\n\nContains all hardware-specific configuration including:\n  - CAN IDs for all drive and turn motors\n  - DIO ports for absolute encoders\n  - Physical dimensions (trackwidth, wheelbase)\n  - Control parameters (speed limits, alignment tolerances)\n  - Wheel positions and rotation angles\n\nEdit this file to match your robot's hardware setup.\nAll distances are in centimeters.\n\"\"\"\n\nimport sys\nimport os\nfrom typing import Dict, Any\n\nsys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))\nfrom CANID import CANID\n\n# Motor and encoder pin assignments with manual offsets and rotation angles\n# Manual offsets represent physical mounting rotation (added to zero calibration)\n# Front wheels are physically 180° rotated from rear wheels\n# Rotation angles are used for 360° in-place spin\nWHEELS: Dict[str, Dict[str, Any]] = {
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
DRIVE_ANGLE_TOLERANCE = 5.0

# Odometry - physical robot measurement
# Trackwidth = left-to-right center-to-center wheel distance (cm)
# Wheelbase  = front-to-rear center-to-center wheel distance (cm)
ROBOT_TRACKWIDTH_CM = 56.0   # left <-> right
ROBOT_WHEELBASE_CM  = 53.0   # front <-> rear

OFFSET_FILE = "/home/lvuser/encoder_offsets.json"
