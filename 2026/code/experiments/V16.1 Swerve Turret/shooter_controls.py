"""
Control bindings that map joystick buttons to subsystem functions.
This file bridges CopilotJoystick and ShooterSubsystem.
"""

from copilotJoystick import CopilotJoystick
from shooter import ShooterSubsystem


class ShooterControls:
	"""Maps joystick buttons to shooter subsystem actions"""
	
	# Motor speed settings
	BUMPER_ROTATION_SPEED = 0.1  # Speed for bumper-triggered rotations (0.0 to 1.0)
	MANUAL_ROTATION_SPEED = 0.1  # Max speed for manual stick rotation (0.0 to 1.0)
	
	def __init__(self, shooter_subsystem, copilot_joystick):
		"""
		Initialize control bindings.
		
		Args:
			shooter_subsystem: ShooterSubsystem instance
			copilot_joystick: CopilotJoystick instance
		"""
		self.shooter = shooter_subsystem
		self.joystick = copilot_joystick
		self.setup_bindings()
	
	def setup_bindings(self):
		"""
		Define all input to function mappings.
		ALL CONTROLS ARE HERE - one central location.
		"""
		# Digital Buttons (triggered once when pressed)
		button_map = {
			"Y": self.uptake_and_shooter,
			"B": self.uptake_and_shooter_reverse,
			"X": self.stop_all,
			"LEFT_BUMPER": self.rotate_to_350,
			"RIGHT_BUMPER": self.rotate_to_15,
		}
		self.joystick.set_button_map(button_map)
		
		# Analog axes (continuous input)
		self.analog_map = {
			"LEFT_X": self.update_turret,
		}
	
	def execute(self):
		"""Execute control logic - call this every cycle"""
		# Process continuous angle-based rotation (if active)
		self.shooter.update_rotation()
		
		# Process analog stick inputs
		for axis_name, function in self.analog_map.items():
			function()
		
		# Process all button presses
		self.joystick.process_buttons()
	
	# ==================== BUTTON FUNCTIONS ====================
	def uptake_and_shooter(self):
		"""Y button: activate uptake and shooter"""
		self.shooter.set_uptake(0.5)
		self.shooter.set_shooter(1.0)
	
	def uptake_and_shooter_reverse(self):
		"""B button: reverse uptake and shooter"""
		self.shooter.set_uptake(-0.2)
		self.shooter.set_shooter(-0.2)
	
	def stop_all(self):
		"""X button: stop all motors"""
		self.shooter.stop_all()

	def rotate_to_350(self):
		"""Left bumper: rotate turret to 350 degrees (left limit check)."""
		self.shooter.rotate_to_angle(350, speed=self.BUMPER_ROTATION_SPEED)

	def rotate_to_15(self):
		"""Right bumper: rotate turret to 15 degrees (right limit check)."""
		self.shooter.rotate_to_angle(15, speed=self.BUMPER_ROTATION_SPEED)
	
	# ==================== ANALOG FUNCTIONS ====================
	def update_turret(self):
		"""Left X axis: control turret rotation with limit checking"""
		turret_speed = self.joystick.get_left_x()
		
		# Scale turret speed with configurable max power
		scaled_speed = turret_speed * self.MANUAL_ROTATION_SPEED
		
		if abs(scaled_speed) > 0.05:  # Dead zone to prevent jitter
			self.shooter.set_turret(scaled_speed)
		else:
			# Joystick in dead zone - stop turret
			if self.shooter.turret:
				self.shooter.turret.stop()
