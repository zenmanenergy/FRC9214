"""
Control bindings that map joystick buttons to subsystem functions.
This file bridges CopilotJoystick and ShooterSubsystem.
"""

from copilotJoystick import CopilotJoystick
from shooter import ShooterSubsystem


class ShooterControls:
	"""Maps joystick buttons to shooter subsystem actions"""
	
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
			# "A": self.function_a,
			# "LEFT_BUMPER": self.function_left_bumper,
			# "RIGHT_BUMPER": self.function_right_bumper,
			# "BACK": self.function_back,
			# "START": self.function_start,
			# "LEFT_STICK": self.function_left_stick,
			# "RIGHT_STICK": self.function_right_stick,
		}
		self.joystick.set_button_map(button_map)
		
		# Analog axes (continuous input)
		self.analog_map = {
			"LEFT_X": self.update_turret,
			# "LEFT_Y": self.function_left_y,
			# "RIGHT_X": self.function_right_x,
			# "RIGHT_Y": self.function_right_y,
			# "LEFT_TRIGGER": self.function_left_trigger,
			# "RIGHT_TRIGGER": self.function_right_trigger,
		}
	
	def execute(self):
		"""Execute control logic - call this every cycle"""
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
	# ==================== ANALOG FUNCTIONS ====================
	def update_turret(self):
		"""Left X axis: control turret rotation"""
		turret_speed = self.joystick.get_left_x()
		self.shooter.set_turret(turret_speed)
	
	# def function_left_y(self):
	#     """Left Y axis: description"""
	#     pass
	# =========================================================
