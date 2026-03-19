"""
Control bindings that map joystick inputs to swerve drive functions.
This file bridges PilotJoystick and SwerveDrive.
All swerve drive control logic is independent of joystick hardware.
"""

import swerve_config as config
from swerve_spin import drive_simple

class PilotControls:
	"""Maps pilot joystick inputs to swerve drive actions"""
	
	CLIMBER_SPEED = 0.3  # Speed for climber motor (5% as requested)

	def __init__(self, swerve_drive, pilot_joystick, climber):
		"""
		Initialize control bindings.
		
		Args:
			swerve_drive: SwerveDrive instance
			pilot_joystick: PilotJoystick instance
		"""
		self.drive = swerve_drive
		self.joystick = pilot_joystick
		self.climber = climber
		
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

		self.analog_map = {
			"LEFT_TRIGGER": self.climber_down,
			"RIGHT_TRIGGER": self.climber_up
		}
	
	def execute_test(self, active_wheel=None):
		"""
		Execute control logic for test mode - call every cycle
		
		Args:
			active_wheel: Wheel name if one is focused, None otherwise (e.g., from web dashboard)
		"""
		# Process button presses to handle focusing
		self.joystick.process_buttons()
		
		# Use web dashboard focus if set, otherwise use joystick focus
		wheel_to_control = active_wheel if active_wheel else self.focused
		
		# Joystick control
		left_y = self.joystick.get_left_y()  # Drive motor
		right_x = self.joystick.get_right_x()  # Turn motor
		

		if wheel_to_control:
			# FOCUS MODE: Individual wheel control for calibration
			if left_y != 0:
				self.drive.set_wheel_drive_power(wheel_to_control, left_y * config.MOTOR_SCALE_MANUAL)
			else:
				self.drive.set_wheel_drive_power(wheel_to_control, 0.0)

			if right_x != 0:
				self.drive.set_wheel_turn_power(wheel_to_control, right_x * config.MOTOR_SCALE_MANUAL)
			else:
				self.drive.set_wheel_turn_power(wheel_to_control, 0.0)
			# Track angle changes (for potential future use)
			angle = self.drive.get_wheel_angle(wheel_to_control)
			if angle != self.last_printed_angle:
				self.last_printed_angle = angle
		else:
			# No focus - allow swerve teleoperation
			strafe = self.joystick.get_left_x()
			
			# Apply swerve kinematics if any input
			if abs(left_y) > 0.1 or abs(strafe) > 0.1 or abs(right_x) > 0.1:
				self.drive.drive_swerve(left_y, strafe, right_x)
	
	def execute_teleop(self):
		"""Execute control logic for teleop mode - call every cycle"""
		# Get joystick inputs
		for axis_name, function in self.analog_map.items():
			function()
		left_y = self.joystick.get_left_y()  # Forward/backward
		left_x = self.joystick.get_left_x()  # Strafe left/right
		right_x = self.joystick.get_right_x()  # Right stick X
		right_y = self.joystick.get_right_y()  # Right stick Y
		
		# Detect active stick regions
		left_stick_active = abs(left_y) > 0.1 or abs(left_x) > 0.1
		right_stick_active = abs(right_x) > 0.1 or abs(right_y) > 0.1
		
		if right_stick_active:
			# Right stick controls swerve_spin (360° rotation mode)
			drive_simple(self.drive, self.joystick)
			self.had_teleop_input = True
		
		elif left_stick_active:
			# Left stick controls normal swerve drive
			self.drive.drive_swerve(left_y, left_x, 0)  # rotation=0 (no right stick)
			self.had_teleop_input = True
			self.drive.update_single_wheel_alignment()
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
	
	#======================== CLIMBER FUNCTIONS ====================

	def climber_up(self):
		right_speed = self.joystick.get_right_trigger()*self.CLIMBER_SPEED
		left_speed = -self.joystick.get_left_trigger()*self.CLIMBER_SPEED
		#print("climber speed", climber_speed)
		if abs(right_speed) > 0.1:
			self.climber.set_climber(right_speed)
		if abs(left_speed) < 0.1 and abs(right_speed) < 0.1:
			self.climber.set_climber(0)


	def climber_down(self):
		right_speed = self.joystick.get_right_trigger()*self.CLIMBER_SPEED
		left_speed = -self.joystick.get_left_trigger()*self.CLIMBER_SPEED
		#print("climber down", climber_speed)
		if abs(left_speed) > 0.1 and abs(right_speed) <0.1:
			self.climber.set_climber(left_speed)

	