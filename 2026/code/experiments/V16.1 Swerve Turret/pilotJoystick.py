from wpilib import Joystick


class PilotJoystick:
	"""
	Pilot joystick handler for primary swerve drive control.
	Handles Xbox/Logitech joystick input with deadband processing.
	This is independent of the swerve drive logic itself.
	"""
	
	def __init__(self, port=0, deadband=0.1):
		"""
		Initialize the pilot joystick handler.
		
		Args:
			port: Joystick port (default 0)
			deadband: Deadband threshold for axes (default 0.1)
		"""
		self.joystick = Joystick(port)
		self.deadband = deadband
		self.button_map = {}
		self.last_button_states = {}
	
	def apply_deadband(self, value):
		"""
		Apply deadband to a joystick axis value.
		
		Args:
			value: Raw axis value (-1.0 to 1.0)
			
		Returns:
			Value with deadband applied, or 0 if within deadband range
		"""
		if abs(value) < self.deadband:
			return 0.0
		
		# Scale output to use full range from deadband to 1.0
		if value > 0:
			return (value - self.deadband) / (1.0 - self.deadband)
		else:
			return (value + self.deadband) / (1.0 - self.deadband)
	
	# ==================== ANALOG AXES ====================
	def get_left_y(self):
		"""Get left Y axis with deadband (forward/backward)"""
		raw_value = -self.joystick.getRawAxis(1)  # Negative because up is positive
		return self.apply_deadband(raw_value)
	
	def get_left_x(self):
		"""Get left X axis with deadband (strafe left/right)"""
		raw_value = -self.joystick.getRawAxis(0)  # Negate for correct direction
		return self.apply_deadband(raw_value)
	
	def get_right_x(self):
		"""Get right X axis with deadband (rotation left/right)"""
		raw_value = self.joystick.getRawAxis(4)
		return self.apply_deadband(raw_value)
	
	def get_right_y(self):
		"""Get right Y axis with deadband"""
		raw_value = self.joystick.getRawAxis(5)
		return self.apply_deadband(raw_value)
	
	def get_left_trigger(self):
		"""Get left trigger (0.0 to 1.0)"""
		return self.joystick.getRawAxis(2)
	
	def get_right_trigger(self):
		"""Get right trigger (0.0 to 1.0)"""
		return self.joystick.getRawAxis(3)
	
	# ==================== DIGITAL BUTTONS ====================
	def get_a_button(self):
		"""Get A button state"""
		return self.joystick.getRawButton(1)
	
	def get_a_button_pressed(self):
		"""Get A button pressed (once per press)"""
		return self.joystick.getRawButtonPressed(1)
	
	def get_b_button(self):
		"""Get B button state"""
		return self.joystick.getRawButton(2)
	
	def get_b_button_pressed(self):
		"""Get B button pressed (once per press)"""
		return self.joystick.getRawButtonPressed(2)
	
	def get_x_button(self):
		"""Get X button state"""
		return self.joystick.getRawButton(3)
	
	def get_x_button_pressed(self):
		"""Get X button pressed (once per press)"""
		return self.joystick.getRawButtonPressed(3)
	
	def get_y_button(self):
		"""Get Y button state"""
		return self.joystick.getRawButton(4)
	
	def get_y_button_pressed(self):
		"""Get Y button pressed (once per press)"""
		return self.joystick.getRawButtonPressed(4)
	
	def get_left_bumper(self):
		"""Get left bumper state"""
		return self.joystick.getRawButton(5)
	
	def get_left_bumper_pressed(self):
		"""Get left bumper pressed (once per press)"""
		return self.joystick.getRawButtonPressed(5)
	
	def get_right_bumper(self):
		"""Get right bumper state"""
		return self.joystick.getRawButton(6)
	
	def get_right_bumper_pressed(self):
		"""Get right bumper pressed (once per press)"""
		return self.joystick.getRawButtonPressed(6)
	
	def get_back_button(self):
		"""Get back button state"""
		return self.joystick.getRawButton(7)
	
	def get_back_button_pressed(self):
		"""Get back button pressed (once per press)"""
		return self.joystick.getRawButtonPressed(7)
	
	def get_start_button(self):
		"""Get start button state"""
		return self.joystick.getRawButton(8)
	
	def get_start_button_pressed(self):
		"""Get start button pressed (once per press)"""
		return self.joystick.getRawButtonPressed(8)
	
	def get_left_stick_button(self):
		"""Get left stick button (press) state"""
		return self.joystick.getRawButton(9)
	
	def get_left_stick_button_pressed(self):
		"""Get left stick button pressed (once per press)"""
		return self.joystick.getRawButtonPressed(9)
	
	def get_right_stick_button(self):
		"""Get right stick button (press) state"""
		return self.joystick.getRawButton(10)
	
	def get_right_stick_button_pressed(self):
		"""Get right stick button pressed (once per press)"""
		return self.joystick.getRawButtonPressed(10)
	
	# ==================== BUTTON HANDLER MANAGEMENT ====================
	def set_button_map(self, button_map):
		"""
		Set button to function mappings.
		
		Args:
			button_map: Dictionary mapping button names to callable functions
				Example: {
					"Y": self.activate_something,
					"A": self.do_something
				}
		"""
		self.button_map = button_map
	
	def process_buttons(self):
		"""
		Process all mapped button presses and execute their functions.
		"""
		button_states = {
			"A": self.get_a_button_pressed(),
			"B": self.get_b_button_pressed(),
			"X": self.get_x_button_pressed(),
			"Y": self.get_y_button_pressed(),
			"LEFT_BUMPER": self.get_left_bumper_pressed(),
			"RIGHT_BUMPER": self.get_right_bumper_pressed(),
			"BACK": self.get_back_button_pressed(),
			"START": self.get_start_button_pressed(),
			"LEFT_STICK": self.get_left_stick_button_pressed(),
			"RIGHT_STICK": self.get_right_stick_button_pressed(),
		}
		
		# Execute function for any pressed button that has a mapped function
		for button_name, is_pressed in button_states.items():
			if is_pressed and button_name in self.button_map:
				function = self.button_map[button_name]
				function()
	
	def get_raw_joystick(self):
		"""Get raw Joystick for direct access if needed"""
		return self.joystick
