"""Calibration Mode Handler - Manages dashboard control and calibration mode logic"""
from wpilib import SmartDashboard
import json
from swerve.odometry_calibrator import RotationCalibrator, TranslationCalibrator


class CalibrationModeHandler:
	"""Handle all calibration mode dashboard commands and control logic"""
	
	def __init__(self, drive, pilot_controls, navigator=None):
		"""Initialize calibration mode handler
		
		Args:
			drive: SwerveDrive instance
			pilot_controls: PilotControls instance
			navigator: WaypointNavigator instance (optional, for autotune)
		"""
		self.drive = drive
		self.pilot_controls = pilot_controls
		self.navigator = navigator
		
		# State tracking for command detection
		self.last_focused_wheel = None
		self.last_calibrate_wheel = None
		self.last_set_wheel_angle_name = None
		self.driving_wheel_to_angle = None
		self.driving_wheel_target_angle = None
		self.autotune_rotation_active = False
		
		# Odometry + IMU calibration routines (see docs/plans/odometry_imu_calibration_plan.md)
		self.rotation_calibrator = RotationCalibrator(drive, drive.calibration)
		self.translation_calibrator = TranslationCalibrator(drive, drive.calibration)
		
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
		
		# Handle rotation autotune if active
		if self.autotune_rotation_active and self.navigator:
			# Autotune runs asynchronously - we just skip other controls
			SmartDashboard.putString("autotune_status", "Tuning rotation...")
			return
		
		# Odometry/IMU calibration routines run exclusively - see
		# docs/plans/odometry_imu_calibration_plan.md section 9.
		self._handle_rotation_calibration_command()
		self._handle_translation_calibration_command()
		self._publish_calibration_status()
		if self.rotation_calibrator.state != "idle" or self.translation_calibrator.state != "idle":
			self.rotation_calibrator.update()
			self.translation_calibrator.update()
			return
		
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
		if hasattr(self.drive, 'tuner') and self.drive.tuner.is_active():
			self._update_autotune_status()
			self.drive.tuner.update()
		
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
		autotune_rotation_command = SmartDashboard.getBoolean("autotune_rotation_command", False)
		tuning_history_command = SmartDashboard.getBoolean("tuning_history_command", False)
		clear_tuning_command = SmartDashboard.getBoolean("clear_tuning_history_command", False)
		
		if autotune_command:
			self.drive.start_autotune()
			SmartDashboard.putBoolean("autotune_command", False)
		
		if autotune_rotation_command and self.navigator:
			self.autotune_rotation_active = True
			SmartDashboard.putBoolean("autotune_rotation_command", False)
			print("[HANDLER] Starting rotation autotune...", flush=True)
			# Run autotune - this blocks for ~15-20 seconds
			result = self.navigator.autotune_rotation(
				target_heading=45.0,
				max_power=0.5,
				duration_seconds=15.0
			)
			self.autotune_rotation_active = False
			
			# Send result back to dashboard
			if result.get('success'):
				SmartDashboard.putString("autotune_rotation_status", f"✓ Complete: KP={result['kp']:.6f}")
				print(f"[HANDLER] Rotation autotune complete: {result}", flush=True)
			else:
				SmartDashboard.putString("autotune_rotation_status", f"✗ Failed: {result.get('message', 'Unknown error')}")
				print(f"[HANDLER] Rotation autotune failed: {result.get('message')}", flush=True)
		
		if tuning_history_command:
			self.drive._publish_tuning_history_to_nt()
			SmartDashboard.putBoolean("tuning_history_command", False)
		
		if clear_tuning_command:
			self.drive.calibration.clear_tuning_history()
			SmartDashboard.putBoolean("clear_tuning_history_command", False)
	
	def _update_autotune_status(self):
		"""Update autotune status on NetworkTables"""
		if hasattr(self.drive, 'tuner') and hasattr(self.drive.tuner, 'gains') and self.drive.tuner.gains:
			wheel_name = self.drive.tuner.gains["wheels"][self.drive.tuner.gains["current_index"]]
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

	# ------------------------------------------------------------------
	# Odometry + IMU calibration (docs/plans/odometry_imu_calibration_plan.md)
	# ------------------------------------------------------------------

	def _handle_rotation_calibration_command(self):
		"""Handle N-spin rotation calibration commands from the dashboard."""
		cal = self.rotation_calibrator

		if SmartDashboard.getBoolean("rotcal_start_command", False):
			SmartDashboard.putBoolean("rotcal_start_command", False)
			n = SmartDashboard.getNumber("rotcal_n", 3)
			speed_pct = SmartDashboard.getNumber("rotcal_speed_pct", 50)
			cal.start_trial(n, speed_pct)

		if SmartDashboard.getBoolean("rotcal_submit_residual_command", False):
			SmartDashboard.putBoolean("rotcal_submit_residual_command", False)
			residual_deg = SmartDashboard.getNumber("rotcal_residual_deg", 0.0)
			if cal.state == "awaiting_input":
				cal.submit_residual(residual_deg)

		if SmartDashboard.getBoolean("rotcal_confirm_reset_command", False):
			SmartDashboard.putBoolean("rotcal_confirm_reset_command", False)
			cal.confirm_reset()

		if SmartDashboard.getBoolean("rotcal_cancel_command", False):
			SmartDashboard.putBoolean("rotcal_cancel_command", False)
			cal.cancel()

		if SmartDashboard.getBoolean("rotcal_apply_command", False):
			SmartDashboard.putBoolean("rotcal_apply_command", False)
			self.drive.calibration.save_calibration()
			cal.reset_session()

		if SmartDashboard.getBoolean("rotcal_discard_command", False):
			SmartDashboard.putBoolean("rotcal_discard_command", False)
			self.drive.calibration.discard_odometry_calibration_changes()
			self.drive.odometry.load_calibration(self.drive.calibration)
			self.drive.imu.set_scale_factor(self.drive.calibration.get_rotation_calibration()["imu_scale_factor"])
			cal.reset_session()

		if SmartDashboard.getBoolean("rotcal_set_accuracy_target_command", False):
			SmartDashboard.putBoolean("rotcal_set_accuracy_target_command", False)
			target = SmartDashboard.getNumber("rotcal_accuracy_target_deg", 2.0)
			self.drive.calibration.set_rotation_accuracy_target(target)

	def _handle_translation_calibration_command(self):
		"""Handle 7-level translation calibration commands from the dashboard."""
		cal = self.translation_calibrator

		if SmartDashboard.getBoolean("transcal_start_command", False):
			SmartDashboard.putBoolean("transcal_start_command", False)
			level = int(SmartDashboard.getNumber("transcal_level", 1))
			x_meters = SmartDashboard.getNumber("transcal_x_meters", 1.0)
			speed_pct = SmartDashboard.getNumber("transcal_speed_pct", 50)
			cal.start_trial(level, x_meters, speed_pct)

		if SmartDashboard.getBoolean("transcal_submit_command", False):
			SmartDashboard.putBoolean("transcal_submit_command", False)
			if cal.state == "awaiting_input":
				measured_distance_m = SmartDashboard.getNumber("transcal_measured_distance_m", cal.x_meters)
				perp_drift_cm = SmartDashboard.getNumber("transcal_perp_drift_cm", 0.0)
				measured_x_cm = SmartDashboard.getNumber("transcal_measured_x_cm", 0.0)
				measured_y_cm = SmartDashboard.getNumber("transcal_measured_y_cm", 0.0)
				cal.submit_result(
					measured_distance_m=measured_distance_m,
					perpendicular_drift_cm=perp_drift_cm,
					measured_x_cm=measured_x_cm,
					measured_y_cm=measured_y_cm,
				)

		if SmartDashboard.getBoolean("transcal_confirm_reset_command", False):
			SmartDashboard.putBoolean("transcal_confirm_reset_command", False)
			cal.confirm_reset()

		if SmartDashboard.getBoolean("transcal_cancel_command", False):
			SmartDashboard.putBoolean("transcal_cancel_command", False)
			cal.cancel()

		if SmartDashboard.getBoolean("transcal_apply_command", False):
			SmartDashboard.putBoolean("transcal_apply_command", False)
			self.drive.calibration.save_calibration()
			cal.reset_session()

		if SmartDashboard.getBoolean("transcal_discard_command", False):
			SmartDashboard.putBoolean("transcal_discard_command", False)
			self.drive.calibration.discard_odometry_calibration_changes()
			self.drive.odometry.load_calibration(self.drive.calibration)
			cal.reset_session()

		if SmartDashboard.getBoolean("transcal_set_accuracy_target_command", False):
			SmartDashboard.putBoolean("transcal_set_accuracy_target_command", False)
			target = SmartDashboard.getNumber("transcal_accuracy_target_pct", 1.5)
			self.drive.calibration.set_translation_accuracy_target(target)

	def _publish_calibration_status(self):
		"""Publish both calibrators' status as JSON for the dashboard to poll."""
		SmartDashboard.putString("rotation_calibration_status", json.dumps(self.rotation_calibrator.status()))
		SmartDashboard.putString("translation_calibration_status", json.dumps(self.translation_calibrator.status()))

