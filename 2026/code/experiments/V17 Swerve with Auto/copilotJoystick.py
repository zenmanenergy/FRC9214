from wpilib import Joystick


class CopilotJoystick:
	def __init__(self, port=0, deadband=0.1, button_map=None):
		"""
		Initialize the copilot joystick handler.
		
		Args:
			port: Controller port (default 0)
			deadband: Deadband threshold (default 0.1)
			button_map: Dictionary mapping button names to functions
		"""
		self.joystick = Joystick(port)
		self.deadband = deadband
		self.button_map = button_map if button_map is not None else {}
		# Track previous button state for edge detection
		self.prev_button_states = {}
	
	def safe_get_axis(self, axis_num):
		"""
		Safely get a joystick axis, returning 0.0 if not available.
		
		Args:
			axis_num: The axis index (0-5 for standard controller)
			
		Returns:
			Axis value (-1.0 to 1.0) or 0.0 if axis not available
		"""
		try:
			return self.joystick.getRawAxis(axis_num)
		except (IndexError, RuntimeError):
			# Controller not plugged in or axis not available
			return 0.0
	
	def safe_get_button(self, button_num):
		"""
		Safely get a joystick button, returning False if not available.
		
		Args:
			button_num: The button index (1-10 for standard controller)
			
		Returns:
			Button state (True if pressed) or False if button not available
		"""
		try:
			return self.joystick.getRawButton(button_num)
		except (IndexError, RuntimeError):
			# Controller not plugged in or button not available
			return False
	
	def is_available(self):
		"""
		Check if the joystick controller is plugged in and available.
		
		Returns:
			True if controller is available, False otherwise
		"""
		try:
			# Try to read axis 0 - if it succeeds, controller is available
			self.joystick.getRawAxis(0)
			return True
		except (IndexError, RuntimeError):
			return False
	
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
		raw_value = self.safe_get_axis(0)
		return self.apply_deadband(raw_value)
	
	def get_left_x(self):
		"""Get left X axis with deadband"""
		raw_value = self.safe_get_axis(0)
		return self.apply_deadband(raw_value)
	
	def get_arm_speed(self):
		"""Get arm speed from right Y axis (inverted) with deadband"""
		raw_value = -self.safe_get_axis(5)
		return self.apply_deadband(raw_value)
	
	def get_y_button(self):
		"""Get Y button state"""
		return self.safe_get_button(4)
	
	def get_raw_joystick(self):
		"""Get raw Joystick for direct access if needed"""
		return self.joystick
	
	# ==================== ADDITIONAL AXES ====================
	def get_left_y(self):
		"""Get left Y axis with deadband"""
		raw_value = -self.safe_get_axis(1)
		return self.apply_deadband(raw_value)
	
	def get_right_x(self):
		"""Get right X axis with deadband"""
		raw_value = self.safe_get_axis(4)
		return self.apply_deadband(raw_value)
	
	def get_right_y(self):
		"""Get right Y axis with deadband"""
		raw_value = -self.safe_get_axis(5)
		return self.apply_deadband(raw_value)
	
	def get_left_trigger(self):
		"""Get left trigger (0.0 to 1.0)"""
		return self.safe_get_axis(2)
	
	def get_right_trigger(self):
		"""Get right trigger (0.0 to 1.0)"""
		return self.safe_get_axis(3)
	
	# ==================== DIGITAL BUTTONS ====================
	def get_a_button(self):
		"""Get A button state"""
		return self.safe_get_button(1)
	
	def get_b_button(self):
		"""Get B button state"""
		return self.safe_get_button(2)
	
	def get_x_button(self):
		"""Get X button state"""
		return self.safe_get_button(3)
	
	def get_left_bumper(self):
		"""Get left bumper state"""
		return self.safe_get_button(5)
	
	def get_right_bumper(self):
		"""Get right bumper state"""
		return self.safe_get_button(6)
	
	def get_back_button(self):
		"""Get back button state"""
		return self.safe_get_button(7)
	
	def get_start_button(self):
		"""Get start button state"""
		return self.safe_get_button(8)
	
	def get_left_stick_button(self):
		"""Get left stick button (press) state"""
		return self.safe_get_button(9)
	
	def get_right_stick_button(self):
		"""Get right stick button (press) state"""
		return self.safe_get_button(10)
	
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
		Only triggers on press edge (transition from unpressed to pressed).
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
		
		# Execute function for any button that just transitioned to pressed
		for button_name, is_pressed in button_states.items():
			prev_pressed = self.prev_button_states.get(button_name, False)
			
			# Debug: Show bumper state changes
			if button_name in ["LEFT_BUMPER", "RIGHT_BUMPER"] and is_pressed != prev_pressed:
				print(f"[COPILOT] {button_name} state changed: {prev_pressed} -> {is_pressed}", flush=True)
			
			# Only trigger on rising edge (False -> True transition)
			if is_pressed and not prev_pressed and button_name in self.button_map:
				print(f"[COPILOT] Executing {button_name} button function", flush=True)
				function = self.button_map[button_name]
				function()
			
			# Store current state for next cycle
			self.prev_button_states[button_name] = is_pressed
