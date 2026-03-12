from wpilib import XboxController


class CopilotJoystick:
	def __init__(self, port=0, deadband=0.1, button_map=None):
		"""
		Initialize the copilot joystick handler.
		
		Args:
			port: Controller port (default 0)
			deadband: Deadband threshold (default 0.1)
			button_map: Dictionary mapping button names to functions
		"""
		self.joystick = XboxController(port)
		self.deadband = deadband
		self.button_map = button_map if button_map is not None else {}
	
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
	
	def get_turret_speed(self):
		"""Get turret speed from left X axis with deadband"""
		raw_value = self.joystick.getLeftX()
		return self.apply_deadband(raw_value)
	
	def get_left_x(self):
		"""Get left X axis with deadband"""
		raw_value = self.joystick.getLeftX()
		return self.apply_deadband(raw_value)
	
	def get_arm_speed(self):
		"""Get arm speed from right Y axis (inverted) with deadband"""
		raw_value = -self.joystick.getRightY()
		return self.apply_deadband(raw_value)
	
	def get_y_button(self):
		"""Get Y button state"""
		return self.joystick.getYButton()
	
	def get_raw_joystick(self):
		"""Get raw XboxController for direct access if needed"""
		return self.joystick
	
	# ==================== ADDITIONAL AXES ====================
	def get_left_y(self):
		"""Get left Y axis with deadband"""
		raw_value = self.joystick.getLeftY()
		return self.apply_deadband(raw_value)
	
	def get_right_x(self):
		"""Get right X axis with deadband"""
		raw_value = self.joystick.getRightX()
		return self.apply_deadband(raw_value)
	
	def get_left_trigger(self):
		"""Get left trigger (0.0 to 1.0)"""
		return self.joystick.getLeftTriggerAxis()
	
	def get_right_trigger(self):
		"""Get right trigger (0.0 to 1.0)"""
		return self.joystick.getRightTriggerAxis()
	
	# ==================== DIGITAL BUTTONS ====================
	def get_a_button(self):
		"""Get A button state"""
		return self.joystick.getAButton()
	
	def get_b_button(self):
		"""Get B button state"""
		return self.joystick.getBButton()
	
	def get_x_button(self):
		"""Get X button state"""
		return self.joystick.getXButton()
	
	def get_left_bumper(self):
		"""Get left bumper state"""
		return self.joystick.getLeftBumper()
	
	def get_right_bumper(self):
		"""Get right bumper state"""
		return self.joystick.getRightBumper()
	
	def get_back_button(self):
		"""Get back button state"""
		return self.joystick.getBackButton()
	
	def get_start_button(self):
		"""Get start button state"""
		return self.joystick.getStartButton()
	
	def get_left_stick_button(self):
		"""Get left stick button (press) state"""
		return self.joystick.getLeftStickButton()
	
	def get_right_stick_button(self):
		"""Get right stick button (press) state"""
		return self.joystick.getRightStickButton()
	
	# ==================== BUTTON HANDLER MANAGEMENT ====================
	def set_button_map(self, button_map):
		"""
		Set button to function mappings.
		
		Args:
			button_map: Dictionary mapping button names to callable functions
				Example: {
					"Y": self.activate_shooter,
					"LEFT_X": self.rotate_turret,
					"A": self.do_something
				}
		"""
		self.button_map = button_map
	
	def process_buttons(self):
		"""
		Process all mapped button presses and execute their functions.
		"""
		button_states = {
			"A": self.get_a_button(),
			"B": self.get_b_button(),
			"X": self.get_x_button(),
			"Y": self.get_y_button(),
			"LEFT_BUMPER": self.get_left_bumper(),
			"RIGHT_BUMPER": self.get_right_bumper(),
			"BACK": self.get_back_button(),
			"START": self.get_start_button(),
			"LEFT_STICK": self.get_left_stick_button(),
			"RIGHT_STICK": self.get_right_stick_button(),
		}
		
		# Execute function for any pressed button that has a mapped function
		for button_name, is_pressed in button_states.items():
			if is_pressed and button_name in self.button_map:
				function = self.button_map[button_name]
				function()