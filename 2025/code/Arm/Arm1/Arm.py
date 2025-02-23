import rev
import wpilib
import json
import os

class Arm:
	PRESETS_FILE = "/home/lvuser/presets.json"  # Persistent file location

	def __init__(self):
		# Motor CAN IDs
		self.ELEVATOR_MOTOR_ID = 10
		self.SHOULDER_MOTOR_ID = 11
		self.WRIST_MOTOR_ID = 12
		self.GRABBER_MOTOR_ID = 13

		# Limit Switch DIO Ports
		self.ELEVATOR_HOME_LIMIT_DIO = 0
		self.SHOULDER_HOME_LIMIT_DIO = 1
		self.WRIST_HOME_LIMIT_DIO = 2

		# Elevator Safe Position Before Homing Other Joints
		self.ELEVATOR_SAFE_HEIGHT = 300  # mm
		self.homing_step = 0  # 0 = Not started, 1 = Elevator, 2 = Shoulder, 3 = Wrist, 4 = Done

		# Create Motors
		self.elevator_motor = rev.CANSparkMax(self.ELEVATOR_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
		self.shoulder_motor = rev.CANSparkMax(self.SHOULDER_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)
		self.wrist_motor = rev.CANSparkMax(self.WRIST_MOTOR_ID, rev.CANSparkMax.MotorType.kBrushless)

		# Configure Motors
		self._configure_motor(self.elevator_motor)
		self._configure_motor(self.shoulder_motor)
		self._configure_motor(self.wrist_motor)

		# Encoders
		self.elevator_encoder = self.elevator_motor.getEncoder()
		self.shoulder_encoder = self.shoulder_motor.getEncoder()
		self.wrist_encoder = self.wrist_motor.getEncoder()

		# Limit Switches
		self.elevator_home_limit = wpilib.DigitalInput(self.ELEVATOR_HOME_LIMIT_DIO)
		self.shoulder_home_limit = wpilib.DigitalInput(self.SHOULDER_HOME_LIMIT_DIO)
		self.wrist_home_limit = wpilib.DigitalInput(self.WRIST_HOME_LIMIT_DIO)

		# Load Presets
		self.preset_positions = self._load_presets()

	def _configure_motor(self, motor):
		"""Configure basic settings for each motor."""
		motor.restoreFactoryDefaults()
		motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
		motor.setSmartCurrentLimit(20)  # Safe current limit
		motor.burnFlash()

	def home_arm(self):
		"""Run homing sequence step-by-step."""
		if self.homing_step == 0:
			print("Homing Elevator...")
			self.homing_step = 1

		elif self.homing_step == 1 and not self.elevator_home_limit.get():
			self.elevator_motor.set(0)
			self.elevator_encoder.setPosition(0)
			print("Elevator homed.")
			self.homing_step = 2

		elif self.homing_step == 2 and self.elevator_encoder.getPosition() >= self.ELEVATOR_SAFE_HEIGHT:
			self.elevator_motor.set(0)
			print("Elevator moved to safe height.")
			self.homing_step = 3

		elif self.homing_step == 3 and not self.shoulder_home_limit.get():
			self.shoulder_motor.set(0)
			self.shoulder_encoder.setPosition(0)
			print("Shoulder homed.")
			self.homing_step = 4

		elif self.homing_step == 4 and not self.wrist_home_limit.get():
			self.wrist_motor.set(0)
			self.wrist_encoder.setPosition(0)
			print("Wrist homed.")
			self.homing_step = 5

	def record_preset(self, name):
		"""Save the current position as a preset."""
		self.preset_positions[name] = {
			"elevator": self.elevator_encoder.getPosition(),
			"shoulder": self.shoulder_encoder.getPosition(),
			"wrist": self.wrist_encoder.getPosition()
		}
		self._save_presets()
		print(f"Preset '{name}' recorded.")

	def move_to_preset(self, name):
		"""Move arm to a recorded preset position."""
		if name in self.preset_positions:
			preset = self.preset_positions[name]
			self.elevator_motor.set(0.1 if preset["elevator"] > self.elevator_encoder.getPosition() else -0.1)
			self.shoulder_motor.set(0.1 if preset["shoulder"] > self.shoulder_encoder.getPosition() else -0.1)
			self.wrist_motor.set(0.1 if preset["wrist"] > self.wrist_encoder.getPosition() else -0.1)
			print(f"Moving to preset '{name}'.")

	def _save_presets(self):
		with open(self.PRESETS_FILE, "w") as f:
			json.dump(self.preset_positions, f)

	def _load_presets(self):
		if os.path.exists(self.PRESETS_FILE):
			with open(self.PRESETS_FILE, "r") as f:
				return json.load(f)
		return {}

	def periodic(self):
		"""Run in main loop."""
		self.home_arm()
