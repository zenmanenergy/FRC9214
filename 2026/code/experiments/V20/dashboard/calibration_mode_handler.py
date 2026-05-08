"""Calibration Mode Handler - Manages dashboard control and calibration mode logic"""
from wpilib import SmartDashboard


class CalibrationModeHandler:
	"""Handle all calibration mode dashboard commands and control logic"""
	
	def __init__(self, drive, pilot_controls):
		"""Initialize calibration mode handler
		
		Args:
			drive: SwerveDrive instance
			pilot_controls: PilotControls instance
		"""
		self.drive = drive
		self.pilot_controls = pilot_controls
		
		# State tracking for command detection
		self.last_focused_wheel = None
		self.last_calibrate_wheel = None
		self.last_set_wheel_angle_name = None
		self.driving_wheel_to_angle = None
		self.driving_wheel_target_angle = None
		
		# Initialize NetworkTables values
		self._init_network_tables()
	
	def _init_network_tables(self):
		"""Initialize NetworkTables with valid defaults"""
		SmartDashboard.putString("calibrate_wheel", "")
		SmartDashboard.putNumber("calibrate_angle", 0)
		SmartDashboard.putString("focused_wheel", "")
		SmartDashboard.putString("robot_mode", "Unknown")
	
	def test_init(self):
		"""Called when calibration mode initializes"""
		SmartDashboard.putString("robot_mode", "Calibration")
	
	def test_periodic(self):
		"""Called periodically during calibration mode"""
		SmartDashboard.putString("robot_mode", "Calibration")
		
		# Read all control commands from NetworkTables
		self._handle_focus_change()
		self._handle_wheel_direction_command()
		self._handle_align_command()
		self._handle_autotune_commands()
		self._handle_calibration_command()
		self._handle_set_wheel_angle_command()
		self._handle_save_zero_command()
		self._handle_continuous_wheel_drive()
		
		# Update drive systems
		self.drive.update_alignment()
		self.drive.update_single_wheel_alignment()
		if self.drive.autotuning:
			self._update_autotune_status()
			self.drive.update_autotune()
		
		# Execute joystick control
		focused_wheel = SmartDashboard.getString("focused_wheel", "")
		self.pilot_controls.execute_test(active_wheel=focused_wheel)
		
		# Update distance tracking
		self.drive.odometry.update()
	
	def test_exit(self):
		"""Called when exiting calibration mode"""
		self.drive.stop_all()
	
	def _handle_focus_change(self):
		"""Detect and handle focused wheel changes from web dashboard"""
		focused_wheel = SmartDashboard.getString("focused_wheel", "")
		
		if focused_wheel != self.last_focused_wheel:
			self.drive.stop_all()
			self.last_focused_wheel = focused_wheel
			# If browser clears focus (empty string), clear pilot focus too
			if not focused_wheel:
				self.pilot_controls.focused = None
	
	def _handle_wheel_direction_command(self):
		"""Handle set_wheels_direction command"""
		set_wheels_direction_str = SmartDashboard.getString("set_wheels_direction", "")
		
		if set_wheels_direction_str:
			import ast
			angles_dict = ast.literal_eval(set_wheels_direction_str)
			focused_wheel_preset = SmartDashboard.getString("focused_wheel_preset", "")
			
			if focused_wheel_preset and focused_wheel_preset in angles_dict:
				target_angle = angles_dict[focused_wheel_preset]
				self.drive.drive_wheel_to_angle(focused_wheel_preset, target_angle)
			else:
				target_angle = list(angles_dict.values())[0] if angles_dict else 0
				self.drive.rotate_to_angle(target_angle)
			
			SmartDashboard.putString("set_wheels_direction", "")
			SmartDashboard.putString("focused_wheel_preset", "")
	
	def _handle_align_command(self):
		"""Handle start alignment command"""
		align_command = SmartDashboard.getBoolean("align_command", False)
		
		if align_command:
			self.drive.start_alignment()
			SmartDashboard.putBoolean("align_command", False)
	
	def _handle_autotune_commands(self):
		"""Handle autotune, tuning history, and clear history commands"""
		autotune_command = SmartDashboard.getBoolean("autotune_command", False)
		tuning_history_command = SmartDashboard.getBoolean("tuning_history_command", False)
		clear_tuning_command = SmartDashboard.getBoolean("clear_tuning_history_command", False)
		
		if autotune_command:
			self.drive.start_autotune()
			SmartDashboard.putBoolean("autotune_command", False)
		
		if tuning_history_command:
			self.drive._publish_tuning_history_to_nt()
			SmartDashboard.putBoolean("tuning_history_command", False)
		
		if clear_tuning_command:
			self.drive.calibration.clear_tuning_history()
			SmartDashboard.putBoolean("clear_tuning_history_command", False)
	
	def _update_autotune_status(self):
		"""Update autotune status on NetworkTables"""
		if self.drive.autotune_gains:
			wheel_name = self.drive.autotune_gains["wheels"][self.drive.autotune_gains["current_index"]]
			SmartDashboard.putString("autotune_wheel", wheel_name)
	
	def _handle_calibration_command(self):
		"""Handle wheel calibration commands"""
		calibrate_wheel = SmartDashboard.getString("calibrate_wheel", "")
		
		if calibrate_wheel and calibrate_wheel != self.last_calibrate_wheel:
			self.last_calibrate_wheel = calibrate_wheel
			calibrate_angle = SmartDashboard.getNumber("calibrate_angle", -999)
			if calibrate_angle >= 0:
				self.drive.set_wheel_angle(calibrate_wheel, int(calibrate_angle))
			SmartDashboard.putString("calibrate_wheel", "")
		elif not calibrate_wheel:
			self.last_calibrate_wheel = None
	
	def _handle_set_wheel_angle_command(self):
		"""Handle set_wheel_angle command from dashboard buttons"""
		set_wheel_angle_name = SmartDashboard.getString("set_wheel_angle_name", "")
		
		if set_wheel_angle_name and set_wheel_angle_name != self.last_set_wheel_angle_name:
			self.last_set_wheel_angle_name = set_wheel_angle_name
			set_wheel_angle_value = SmartDashboard.getNumber("set_wheel_angle_value", -1)
			if set_wheel_angle_value >= 0 and set_wheel_angle_value < 360:
				self.driving_wheel_to_angle = set_wheel_angle_name
				self.driving_wheel_target_angle = int(set_wheel_angle_value)
			SmartDashboard.putString("set_wheel_angle_name", "")
		elif not set_wheel_angle_name:
			self.last_set_wheel_angle_name = None
	
	def _handle_save_zero_command(self):
		"""Handle save zero offset command"""
		save_zero_command = SmartDashboard.getBoolean("save_zero_command", False)
		focused_wheel = SmartDashboard.getString("focused_wheel", "")
		
		if save_zero_command and focused_wheel:
			self.drive.set_wheel_zero(focused_wheel)
			self.drive.stop_all()
			SmartDashboard.putBoolean("save_zero_command", False)
			SmartDashboard.putString("focused_wheel", "")
			self.last_focused_wheel = None
	
	def _handle_continuous_wheel_drive(self):
		"""Handle continuous wheel rotation to target angle"""
		if self.driving_wheel_to_angle:
			current_angle = self.drive.get_wheel_angle(self.driving_wheel_to_angle)
			raw_error = self.driving_wheel_target_angle - current_angle
			
			error = raw_error
			if error > 180:
				error -= 360
			elif error < -180:
				error += 360
			
			DASHBOARD_TOLERANCE = 1.5
			abs_error = abs(error)
			
			if abs_error < DASHBOARD_TOLERANCE:
				self.drive.stop_all()
				self.driving_wheel_to_angle = None
				self.driving_wheel_target_angle = None
