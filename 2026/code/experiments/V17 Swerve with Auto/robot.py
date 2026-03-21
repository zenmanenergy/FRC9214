"""FRC Swerve Drive Test Robot - Encoder Calibration with Shooter"""
import wpilib
from wpilib import SmartDashboard, DriverStation
from swerve_drive import SwerveDrive
# from turret import Turret
from shooter import ShooterSubsystem
from climber import Climber
from copilotJoystick import CopilotJoystick
from shooter_controls import ShooterControls
from pilotJoystick import PilotJoystick
from pilot_controls import PilotControls
from dashboard_updater import DashboardUpdater
from encoder_calibration import EncoderCalibration
import swerve_config as config
import CANID


class Robot(wpilib.TimedRobot):
	def robotInit(self):
		import sys
		
		self.drive = SwerveDrive()
		
		# self.turret = Turret(turn_canid=CANID.TURRET_TURN, encoder_dio=4)
		self.turret = None  # Turret disabled
		
		self.shooter = ShooterSubsystem(turret=None)
		self.climber = Climber()

		
		# Load calibration for both swerve and turret
		self.calibration = EncoderCalibration()
		# turret_offset = self.calibration.get_offset("turret")
		# self.turret.set_zero_offset(turret_offset)
		# turret_left_limit = self.calibration.get_offset("turret_left_limit")
		# turret_right_limit = self.calibration.get_offset("turret_right_limit")
		# self.turret.set_left_limit(turret_left_limit)
		# self.turret.set_right_limit(turret_right_limit)

		
		self.pilot_joystick = PilotJoystick(port=0, deadband=0.1)
		self.pilot_controls = PilotControls(self.drive, self.pilot_joystick, self.climber)
		
		self.copilot_joystick = CopilotJoystick(port=1, deadband=0.1)
		self.shooter_controls = ShooterControls(self.shooter, self.copilot_joystick)
		
		self.dashboard = DashboardUpdater(self.drive, None)
		
		# Test state for web dashboard integration
		self.last_focused_wheel = None
		self.last_calibrate_wheel = None  # Track last calibrated wheel to detect new commands
		self.last_set_wheel_angle_name = None  # Track last wheel angle command to detect new commands
		self.driving_wheel_to_angle = None  # Current wheel being driven to target angle
		self.driving_wheel_target_angle = None  # Target angle for the wheel being driven
		
		# Autonomous mode state machine
		# States: 0=DRIVE_FORWARD, 1=SHOOTING, 2=STOP_ALL, 3=IDLE
		self.auto_state = 0  # Start in IDLE
		self.auto_state_timer = None  # Timer for current state
		self.auto_target_distance_meters = -2.0  # Drive 2 meters backward
		self.auto_shoot_duration = 10.0  # Shoot for 10 seconds
  
		# Initialize NetworkTables values with valid defaults for synchronization
		SmartDashboard.putString("calibrate_wheel", "")
		SmartDashboard.putNumber("calibrate_angle", 0)
		SmartDashboard.putString("focused_wheel", "")
		SmartDashboard.putString("robot_mode", "Unknown")
		SmartDashboard.putBoolean("turret_set_zero_command", False)
		SmartDashboard.putBoolean("turret_set_left_limit_command", False)
		SmartDashboard.putBoolean("turret_set_right_limit_command", False)
		

	
	def testInit(self):
		# Publish mode to NetworkTables so dashboard auto-detects
		# SmartDashboard.putString("robot_mode", "Test")
		pass
	def autonomousInit(self):
		"""Initialize autonomous mode - drive forward then shoot"""
		# Reset odometry to zero
		self.drive.odometry.reset()
		
		# Start in DRIVE_FORWARD state
		self.auto_state = 0  # DRIVE_FORWARD
		self.auto_state_timer = wpilib.Timer.getFPGATimestamp()
		
		# Publish mode to NetworkTables
		# SmartDashboard.putString("robot_mode", "Autonomous")
	
	def autonomousPeriodic(self):
		"""Execute autonomous sequence - drive forward, then shoot, then stop"""
		# Update odometry every loop
		self.drive.odometry.update()
		
		# Get current distance in meters
		distance_meters = self.drive.odometry.get_distance_meters()
		elapsed_time = wpilib.Timer.getFPGATimestamp() - self.auto_state_timer
		
		# State machine
		if self.auto_state == 0:  # DRIVE_BACKWARD
			# Activate Shooter for a set time
			self.shooter.set_shooter(1.0)      # Shooter at 80% speed
			self.shooter.set_uptake(1.0)       # Uptake at 60% speed
			self.shooter.startspindex(0.1)          # Spindexer with 1.0x multiplier

			# Check if we've shot long enough
			if elapsed_time >= self.auto_shoot_duration:
				# Transition to STOP_ALL state
				self.auto_state = 1  # STOP_ALL
				self.auto_state_timer = wpilib.Timer.getFPGATimestamp()
				# Stop all motors
				
			
		
		elif self.auto_state == 1:  # SHOOTING
			# Run uptake and shooter
			self.shooter_controls.uptake_and_shooter()
			self.shooter_controls.stop_all()
			
			
		#Drive at 50% power straight BACKWARD
			self.drive.drive_swerve(forward=-0.5, strafe=0, rotate=0 )
			
			# Check if we've reached target distance
			if distance_meters >= self.auto_target_distance_meters:
				# Stop driving
				self.drive.stop_all()

				# Transition to SHOOTING state
				self.auto_state_timer = wpilib.Timer.getFPGATimestamp()

		# else IDLE - do nothing
	
	def robotPeriodic(self):
		# Update all dashboard values
		self.dashboard.update()
		
		# Publish robot enabled state
		SmartDashboard.putBoolean("Robot Enabled", DriverStation.isEnabled())
	
	def testPeriodic(self):
		# Ensure robot_mode stays set to Test
		SmartDashboard.putString("robot_mode", "Test")
		
		# Read control commands from NetworkTables (from web dashboard)
		focused_wheel = SmartDashboard.getString("focused_wheel", "")
		align_command = SmartDashboard.getBoolean("align_command", False)
		save_zero_command = SmartDashboard.getBoolean("save_zero_command", False)
		calibrate_wheel = SmartDashboard.getString("calibrate_wheel", "")
		calibrate_angle = SmartDashboard.getNumber("calibrate_angle", -999)
		set_wheel_angle_name = SmartDashboard.getString("set_wheel_angle_name", "")
		set_wheel_angle_value = SmartDashboard.getNumber("set_wheel_angle_value", -1)
		set_wheels_direction_str = SmartDashboard.getString("set_wheels_direction", "")
		autotune_command = SmartDashboard.getBoolean("autotune_command", False)
		tuning_history_command = SmartDashboard.getBoolean("tuning_history_command", False)
		clear_tuning_command = SmartDashboard.getBoolean("clear_tuning_history_command", False)
		
		if clear_tuning_command:
			print("[CLEAR-NOW] EXECUTING CLEAR HANDLER!")
		
		# Handle set_wheels_direction command
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
		
		# Detect if focus changed from web dashboard and stop previous wheel
		if focused_wheel != self.last_focused_wheel:
			self.drive.stop_all()
			self.last_focused_wheel = focused_wheel
			# If browser clears focus (empty string), clear pilot focus too
			if not focused_wheel:
				self.pilot_controls.focused = None
		
		# Handle align command
		if align_command:
			self.drive.start_alignment()
			SmartDashboard.putBoolean("align_command", False)
		
		# Handle autotune command
		if autotune_command:
			self.drive.start_autotune()
			SmartDashboard.putBoolean("autotune_command", False)
		
		# Handle tuning history command
		if tuning_history_command:
			self.drive._publish_tuning_history_to_nt()
			SmartDashboard.putBoolean("tuning_history_command", False)
		
		# Handle clear tuning command
		if clear_tuning_command:
			self.drive.calibration.clear_tuning_history()
			SmartDashboard.putBoolean("clear_tuning_history_command", False)
		
		# Update alignment if active
		self.drive.update_alignment()
		
		# Update single-wheel alignment if active
		self.drive.update_single_wheel_alignment()
		
		# Update autotune if active
		if self.drive.autotuning:
			self.drive.update_autotune()
			if self.drive.autotune_gains:
				wheel_name = self.drive.autotune_gains["wheels"][self.drive.autotune_gains["current_index"]]
				SmartDashboard.putString("autotune_wheel", wheel_name)
		
		# Handle calibration command - only on change of calibrate_wheel
		if calibrate_wheel and calibrate_wheel != self.last_calibrate_wheel:
			self.last_calibrate_wheel = calibrate_wheel
			calibrate_angle = SmartDashboard.getNumber("calibrate_angle", -999)
			if calibrate_angle >= 0:
				self.drive.set_wheel_angle(calibrate_wheel, int(calibrate_angle))
			SmartDashboard.putString("calibrate_wheel", "")
		elif not calibrate_wheel:
			self.last_calibrate_wheel = None
		
		# Handle set_wheel_angle command from dashboard buttons
		if set_wheel_angle_name and set_wheel_angle_name != self.last_set_wheel_angle_name:
			self.last_set_wheel_angle_name = set_wheel_angle_name
			set_wheel_angle_value = SmartDashboard.getNumber("set_wheel_angle_value", -1)
			if set_wheel_angle_value >= 0 and set_wheel_angle_value < 360:
				self.driving_wheel_to_angle = set_wheel_angle_name
				self.driving_wheel_target_angle = int(set_wheel_angle_value)
			SmartDashboard.putString("set_wheel_angle_name", "")
		elif not set_wheel_angle_name:
			self.last_set_wheel_angle_name = None
		
		# Handle save zero command
		if save_zero_command and focused_wheel:
			self.drive.set_wheel_zero(focused_wheel)
			self.drive.stop_all()
			SmartDashboard.putBoolean("save_zero_command", False)
			SmartDashboard.putString("focused_wheel", "")
			self.last_focused_wheel = None
		
		# Handle turret set zero command
		# turret_set_zero_command = SmartDashboard.getBoolean("turret_set_zero_command", False)
		turret_set_zero_command = False
		if turret_set_zero_command:
			raw_encoder_degrees = self.turret.get_raw_encoder_degrees()
			# Shift zero point by 180 degrees in angle space
			offset_adjusted = raw_encoder_degrees - (180 * self.turret.GEAR_RATIO)
			self.turret.set_zero_offset(offset_adjusted)
			self.calibration.set_offset("turret", offset_adjusted)
			self.calibration.save_calibration()
			self.turret.stop()
			# SmartDashboard.putBoolean("turret_set_zero_command", False)
		
		# Handle turret set left limit command
		# turret_set_left_limit_command = SmartDashboard.getBoolean("turret_set_left_limit_command", False)
		turret_set_left_limit_command = False
		if turret_set_left_limit_command:
			turret_angle = self.turret.get_angle()
			self.turret.set_left_limit(turret_angle)
			self.calibration.set_offset("turret_left_limit", turret_angle)
			self.calibration.save_calibration()
			self.turret.stop()
			# SmartDashboard.putBoolean("turret_set_left_limit_command", False)
		
		# Handle turret set right limit command
		# turret_set_right_limit_command = SmartDashboard.getBoolean("turret_set_right_limit_command", False)
		turret_set_right_limit_command = False
		if turret_set_right_limit_command:
			turret_angle = self.turret.get_angle()
			self.turret.set_right_limit(turret_angle)
			self.calibration.set_offset("turret_right_limit", turret_angle)
			self.calibration.save_calibration()
			self.turret.stop()
			# SmartDashboard.putBoolean("turret_set_right_limit_command", False)
		
		# Update continuous wheel drive (from dashboard arrow buttons)
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
			else:
				self.drive.drive_wheel_to_angle(self.driving_wheel_to_angle, self.driving_wheel_target_angle)
		
		# Execute joystick-based control (pass focused_wheel from web dashboard to pilot_controls)
		self.pilot_controls.execute_test(active_wheel=focused_wheel)
		
		# Update distance tracking
		self.drive.odometry.update()
		
		# SHOOTER CONTROLS
		self.shooter_controls.execute()
	
	def testExit(self):
		self.drive.stop_all()
	
	def teleopInit(self):
		# Stop all motors on entry for safety
		self.drive.stop_all()
		self.shooter.stop_all()
		
		# Reset swerve wheel alignment state from autonomous
		self.drive.aligning = False
		self.drive.wheel_alignment_state.clear()
		
		# Publish mode to NetworkTables so dashboard auto-detects
		# SmartDashboard.putString("robot_mode", "Teleop")
		
		# Track if robot had input in previous loop
		self.had_teleop_input = False
	
	def teleopPeriodic(self):
		"""Teleop control for swerve drive"""
		# Ensure robot_mode stays set to Teleop
		# SmartDashboard.putString("robot_mode", "Teleop")
		
		# Execute joystick control logic
		self.pilot_controls.execute_teleop()
		
		# Update distance tracking
		self.drive.odometry.update()
		
		# Update single-wheel alignment (handles per-wheel centering)
		self.drive.update_single_wheel_alignment()
		
		# Debug: check if turn motors are truly at 0 power
		#self.drive._debug_print_turn_motor_powers()
		
		# Debug: check drive motor diagnostics
		#self.drive._debug_print_drive_motor_diagnostics()
		

		# SHOOTER CONTROLS
		self.shooter_controls.execute()
	
	def teleopExit(self):
		self.drive.stop_all()
		self.shooter.stop_all()


if __name__ == "__main__":
	wpilib.run(Robot)
