"""
Control bindings that map joystick inputs to swerve drive functions.
This file bridges PilotJoystick and SwerveDrive.
All swerve drive control logic is independent of joystick hardware.
"""

import swerve_config as config

class PilotControls:
	"""Maps pilot joystick inputs to swerve drive actions"""

	def __init__(self, swerve_drive, pilot_joystick):
		"""
		Initialize control bindings.
		
		Args:
			swerve_drive: SwerveDrive instance
			pilot_joystick: PilotJoystick instance
		"""
		self.drive = swerve_drive
		self.joystick = pilot_joystick
		
		# Test mode state
		self.focused = None  # Tracks joystick-selected focus
		self.last_printed_angle = -999
		
		# Teleop state
		self.had_teleop_input = False
		
		self.setup_bindings()
	
	def setup_bindings(self):
		"""
		Define all input to function mappings for test mode.
		ALL CONTROLS ARE HERE - one central location.
		"""
		button_map = {
			"A": lambda: self.toggle_wheel_focus("rear_left"),
			"B": lambda: self.toggle_wheel_focus("rear_right"),
			"X": lambda: self.toggle_wheel_focus("front_left"),
			"Y": lambda: self.toggle_wheel_focus("front_right"),
			"START": self.align_all_wheels,
			"LEFT_BUMPER": lambda: self.save_wheel_angle(90),
			"RIGHT_BUMPER": lambda: self.save_wheel_zero(),
			"BACK": lambda: self.save_wheel_angle(180),
		}
		self.joystick.set_button_map(button_map)
	
	def execute_test(self, active_wheel=None):
		"""
		Execute control logic for test mode - call every cycle
		
		Args:
			active_wheel: Wheel name if one is focused, None otherwise (e.g., from web dashboard)
		"""
		# Run full teleop logic (this tests all wheels together via kinematics)
		self.execute_teleop()
	
	def execute_teleop(self):
		"""Execute control logic for teleop mode - call every cycle"""
		# Get joystick inputs
		left_y = self.joystick.get_left_y()  # Forward/backward
		left_x = self.joystick.get_left_x()  # Strafe left/right
		right_x = self.joystick.get_right_x()  # Right stick rotation
		
		# Detect active stick regions
		left_stick_active = abs(left_y) > 0.1 or abs(left_x) > 0.1
		right_stick_active = abs(right_x) > 0.1
		
		if left_stick_active:
			# Left stick is active - blend movement with rotation using kinematics
			# Pass right stick input for blended rotation, or 0 if not active
			rotate_input = right_x if right_stick_active else 0
			self.drive.drive_swerve(left_y, left_x, rotate_input)
			self.had_teleop_input = True
			self.drive.update_single_wheel_alignment()
			
		elif right_stick_active:
			# Right stick ONLY - use verified rotation angles (45°/135°/225°/315°)
			self.drive.drive_rotation(right_x)
			self.had_teleop_input = True
   
		else:
			# No input - stop motors
			if self.had_teleop_input:
				for wheel_name in self.drive.wheels.keys():
					self.drive.set_wheel_drive_power(wheel_name, 0)
				self.had_teleop_input = False
			
			self.drive.update_single_wheel_alignment()
	
	# ==================== WHEEL FOCUS FUNCTIONS ====================
	def toggle_wheel_focus(self, wheel_name):
		"""Toggle focus on a specific wheel"""
		if self.focused == wheel_name:
			# Toggle off
			self.drive.stop_all()
			self.focused = None
			print(f"[FOCUS] Focused off. Motor stopped.\n")
		else:
			# Focus on new wheel
			self.drive.stop_all()
			self.focused = wheel_name
			self.last_printed_angle = -999
			print(f"[FOCUS] Focused on {wheel_name}. Use left thumb up/down to drive, right thumb left/right to turn.")
	
	# ==================== WHEEL CALIBRATION FUNCTIONS ====================
	def save_wheel_zero(self):
		"""RB: Save current position as zero for focused wheel"""
		if self.focused:
			self.drive.set_wheel_zero(self.focused)
			print(f"[ZEROING] {self.focused} saved as 0deg\n")
	
	def save_wheel_angle(self, angle):
		"""Save current position as specific angle (90, 180) for focused wheel"""
		if self.focused:
			self.drive.set_wheel_angle(self.focused, angle)
			print(f"[ZEROING] {self.focused} saved as {angle}deg\n")
	
	def align_all_wheels(self):
		"""START: Align all wheels to 0deg"""
		self.drive.start_alignment()
	
	def get_focused_wheel(self):
		"""Get the currently focused wheel from joystick"""
		return self.focused
