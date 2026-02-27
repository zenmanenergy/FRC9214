# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
import math
from wpilib import Joystick
from constants import OIConstants
from subsystems.drive_subsystem import DriveSubsystem
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState


class Robot(wpilib.TimedRobot):
	"""
	The VM is configured to automatically run this class, and to call the functions corresponding to
	each mode, as described in the TimedRobot documentation. If you change the name of this class or
	the package after creating this project, you must also update the build.gradle file in the
	project.
	"""

	def robotInit(self):
		"""
		This function is run when the robot is first started up and should be used for any
		initialization code.
		"""
		print("[ROBOT] robotInit() starting...")
		# Initialize subsystems
		self.robot_drive = DriveSubsystem()
		print("[ROBOT] DriveSubsystem initialized")
		
		# Initialize driver joystick (USB port 0) - has dual thumb sticks
		self.driver_joystick = Joystick(OIConstants.k_driver_controller_port)
		print("[ROBOT] Joystick initialized on port 0")
		
		# Initialize autonomous command to None
		self.autonomous_command = None
		print("[ROBOT] robotInit() complete")
		
		# Test mode variables
		self.zeroing_active = False
		self.last_zero_button_state = False

	def robotPeriodic(self):
		"""
		This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
		that you want ran during disabled, autonomous, teleoperated and test.

		This runs after the mode specific periodic functions, but before LiveWindow and
		SmartDashboard integrated updating.
		"""
		# Call drive subsystem's periodic method to apply motor hold commands
		self.robot_drive.periodic()

	def disabledInit(self):
		"""This function is called once each time the robot enters Disabled mode."""
		pass

	def disabledPeriodic(self):
		pass

	def autonomousInit(self):
		"""This autonomous runs the autonomous command selected by your RobotContainer class."""
		# schedule the autonomous command (example)
		if self.autonomous_command is not None:
			self.autonomous_command.schedule()

	def autonomousPeriodic(self):
		"""This function is called periodically during autonomous."""
		pass

	def autonomousExit(self):
		pass

	def teleopInit(self):
		"""
		This makes sure that the autonomous stops running when
		teleop starts running. If you want the autonomous to
		continue until interrupted by another command, remove
		this line or comment it out.
		"""
		print("[ROBOT] === ENTERING TELEOP MODE ===")
		if self.autonomous_command is not None:
			self.autonomous_command.cancel()
		
		# Enable DIO encoder printing for rear left module
		self.robot_drive.rear_left.teleop_enabled = True
		
		# Reset startup flag so diagnostic prints on first cycle
		self.robot_drive.rear_left.startup_printed = False
		
		# Do NOT enable debug by default - it spams console
		# self.robot_drive.set_debug_mode(True)
		
		# Center all wheels on enable (rear left starts at 45° for testing)
		center_state = SwerveModuleState(0, Rotation2d(0))
		#self.robot_drive.front_left.set_desired_state(center_state)
		#self.robot_drive.front_right.set_desired_state(center_state)
		#self.robot_drive.rear_right.set_desired_state(center_state)
		
		# Rear left starts at pi/4 (45 degrees) for DIO encoder testing
		rear_left_state = SwerveModuleState(0, Rotation2d(math.pi / 2))
		self.robot_drive.rear_left.set_desired_state(rear_left_state)
		
		
		print("[ROBOT] Press START + BACK to begin Rear Left turn motor autotune")
		print("[ROBOT] Teleop init complete\n")
		
		self.autotune_triggered = False

	def teleopPeriodic(self):
		"""
		Check for button combinations:
		- START + BACK: trigger autotune
		- Y + X: dump all SparkMax configs
		"""
		try:
			# Check if START + BACK pressed (buttons 8 + 7 on Xbox controller)
			button_start = self.driver_joystick.getRawButton(8)
			button_back = self.driver_joystick.getRawButton(7)
			button_y = self.driver_joystick.getRawButton(4)
			button_x = self.driver_joystick.getRawButton(3)
			
			if button_start and button_back and not self.autotune_triggered:
				print("\n[ROBOT] ===== START + BACK PRESSED - TRIGGERING AUTOTUNE =====")
				print("[ROBOT] STARTING ZIEGLER-NICHOLS AUTOTUNE FOR REAR LEFT TURN MOTOR")
				print("[ROBOT] This will take 8 seconds - motor will oscillate during tuning\n")
				self.robot_drive.rear_left.start_turn_autotune()
				self.autotune_triggered = True
			
			# Check if Y + X pressed to dump SparkMax config
			if button_y and button_x:
				self.robot_drive.print_all_sparkmax_config()
			
			# If autotune just completed, print results once
			if self.autotune_triggered and not self.robot_drive.rear_left.autotune_active:
				kp = self.robot_drive.rear_left.turn_kp
				ki = self.robot_drive.rear_left.turn_ki
				kd = self.robot_drive.rear_left.turn_kd
				
				print("\n" + "="*60)
				print("[ROBOT] ===== AUTOTUNE COMPLETE - RESULTS READY =====")
				print("="*60)
				print(f"[ROBOT] Add these values to constants.py:")
				print(f"[ROBOT] turn_kp  = {kp:.10f}")
				print(f"[ROBOT] turn_ki  = {ki:.10f}")
				print(f"[ROBOT] turn_kd  = {kd:.10f}")
				print("="*60 + "\n")
				
				# Reset trigger so we don't print again
				self.autotune_triggered = False
			
			# Command wheel to center after autotune (or always if not autotuning)
			# DISABLED FOR DIO ENCODER TESTING - rear left position set in teleopInit()
			# if not self.robot_drive.rear_left.autotune_active:
			# 	center_state = SwerveModuleState(0, Rotation2d(0))
			# 	self.robot_drive.rear_left.set_desired_state(center_state)
		
		except Exception as e:
			print(f"[ERROR] teleopPeriodic: {e}")
			import traceback

	def testInit(self):
		"""This function is called once when the robot enters test mode."""
		print("[TEST] === ENTERING TEST MODE ===")
		print("[TEST] Press A to focus on Rear Left wheel")
		print("[TEST] Press X to focus on Front Left wheel")
		# Disable PID immediately to prevent unwanted movement
		self.robot_drive.rear_left.skip_periodic_command = True
		self.robot_drive.front_left.skip_periodic_command = True
		# Focus state
		self.focused_wheel = None  # None, "Rear Left", or "Front Left"
		self.last_a_button_state = False
		self.last_x_button_state = False
		self.last_lb_button_state = False
		# Print only on change
		self.last_printed_angle = None
		# Raw count collection via timer
		self.collecting_raw_counts = False
		self.start_raw_count = None
		self.rotate_start_time = None
		self.rotation_duration = 0.1  # seconds (back to 0.1 for ~45 degree rotation)
		self.loop_counter = 0

	def testPeriodic(self):
		"""This function is called periodically during test mode."""
		# Check A button (rear left focus)
		a_button = self.driver_joystick.getRawButton(1)
		if a_button and not self.last_a_button_state:
			if self.focused_wheel == "Rear Left":
				# Toggle off and print report
				if self.robot_drive.rear_left.dio_encoder:
					offset_value = self.robot_drive.rear_left.dio_encoder.getDistance()
					print("\n" + "="*60)
					print("[TEST] ========== REAR LEFT ZERO OFFSET ==========")
					print(f"[TEST] k_rear_left_encoder_offset = {offset_value}")
					print("="*60 + "\n")
				self.focused_wheel = None
			else:
				# Toggle on
				self.focused_wheel = "Rear Left"
				print("[TEST] REAR LEFT WHEEL - FOCUSED")
		self.last_a_button_state = a_button
		
		# Check X button (front left focus)
		x_button = self.driver_joystick.getRawButton(3)
		if x_button and not self.last_x_button_state:
			if self.focused_wheel == "Front Left":
				# Toggle off and print report
				if self.robot_drive.front_left.dio_encoder:
					offset_value = self.robot_drive.front_left.dio_encoder.getDistance()
					print("\n" + "="*60)
					print("[TEST] ========== FRONT LEFT ZERO OFFSET ==========")
					print(f"[TEST] k_front_left_encoder_offset = {offset_value}")
					print("="*60 + "\n")
				self.focused_wheel = None
			else:
				# Toggle on
				self.focused_wheel = "Front Left"
				print("[TEST] FRONT LEFT WHEEL - FOCUSED")
		self.last_x_button_state = x_button
		
		# Display angle for focused wheel only
		if self.focused_wheel == "Rear Left" and self.robot_drive.rear_left.use_dio_encoder:
			angle_rad = self.robot_drive.rear_left.read_dio_encoder(skip_debounce=True)
			if angle_rad is not None:
				angle_deg = angle_rad * 180.0 / math.pi
				if self.last_printed_angle is None or abs(angle_rad - self.last_printed_angle) > 0.001:
					print(f"[TEST] Rear Left: {angle_rad:.3f} rad ({angle_deg:.1f}°)")
					self.last_printed_angle = angle_rad
		elif self.focused_wheel == "Front Left" and self.robot_drive.front_left.use_dio_encoder:
			angle_rad = self.robot_drive.front_left.read_dio_encoder(skip_debounce=True)
			if angle_rad is not None:
				angle_deg = angle_rad * 180.0 / math.pi
				if self.last_printed_angle is None or abs(angle_rad - self.last_printed_angle) > 0.001:
					print(f"[TEST] Front Left: {angle_rad:.3f} rad ({angle_deg:.1f}°)")
					self.last_printed_angle = angle_rad
		
		# LB button - automatic rotation for 2 seconds with raw count measurement
		lb_button = self.driver_joystick.getRawButton(5)
		
		if lb_button and not self.last_lb_button_state:
			# LB pressed - start automatic rotation
			if self.focused_wheel == "Rear Left" and self.robot_drive.rear_left.dio_encoder:
				self.start_raw_count = self.robot_drive.rear_left.dio_encoder.getRaw()
				self.rotate_start_time = wpilib.Timer.getFPGATimestamp()
				self.collecting_raw_counts = True
				print(f"\n[TEST] START: Rear Left raw_count = {self.start_raw_count}")
			elif self.focused_wheel == "Front Left" and self.robot_drive.front_left.dio_encoder:
				self.start_raw_count = self.robot_drive.front_left.dio_encoder.getRaw()
				self.rotate_start_time = wpilib.Timer.getFPGATimestamp()
				self.collecting_raw_counts = True
				print(f"\n[TEST] START: Front Left raw_count = {self.start_raw_count}")
		
		# Handle automatic rotation timer
		if self.collecting_raw_counts:
			elapsed = wpilib.Timer.getFPGATimestamp() - self.rotate_start_time
			
			if elapsed < self.rotation_duration:
				# Still rotating - apply motor command
				if self.focused_wheel == "Rear Left":
					self.robot_drive.rear_left.turning_spark.set(0.5)  # 50% speed
				elif self.focused_wheel == "Front Left":
					self.robot_drive.front_left.turning_spark.set(0.5)
			else:
				# Done rotating - stop and print result
				if self.focused_wheel == "Rear Left":
					self.robot_drive.rear_left.turning_spark.set(0.0)
					if self.robot_drive.rear_left.dio_encoder:
						end_count = self.robot_drive.rear_left.dio_encoder.getRaw()
						delta = end_count - self.start_raw_count
						cpr = delta / self.rotation_duration
						print(f"[TEST] STOP: Rear Left raw_count = {end_count}")
						print(f"[TEST] Delta: {delta} counts in {self.rotation_duration} sec -> CPR = {cpr:.0f}\n")
				elif self.focused_wheel == "Front Left":
					self.robot_drive.front_left.turning_spark.set(0.0)
					if self.robot_drive.front_left.dio_encoder:
						end_count = self.robot_drive.front_left.dio_encoder.getRaw()
						delta = end_count - self.start_raw_count
						cpr = delta / self.rotation_duration
						print(f"[TEST] STOP: Front Left raw_count = {end_count}")
						print(f"[TEST] Delta: {delta} counts in {self.rotation_duration} sec -> CPR = {cpr:.0f}\n")
				self.collecting_raw_counts = False
		
		self.last_lb_button_state = lb_button
		
		# Joystick control - only for focused wheel (skip if auto-rotating)
		if not self.collecting_raw_counts:
			stick_x = self.driver_joystick.getRawAxis(4)
			# Apply deadband to prevent drift-induced motion
			if abs(stick_x) < 0.05:
				stick_x = 0.0
			rotation_speed = stick_x * 0.2
			
			if self.focused_wheel == "Rear Left":
				if stick_x != 0.0:
					print(f"[TEST] Rear Left motor command: {rotation_speed:.3f} (joystick: {stick_x:.3f})")
				self.robot_drive.rear_left.turning_spark.set(rotation_speed)
			elif self.focused_wheel == "Front Left":
				if stick_x != 0.0:
					print(f"[TEST] Front Left motor command: {rotation_speed:.3f} (joystick: {stick_x:.3f})")
				self.robot_drive.front_left.turning_spark.set(rotation_speed)

	def testExit(self):
		"""Called when exiting test mode."""
		# Re-enable PID when leaving test mode
		self.robot_drive.rear_left.skip_periodic_command = False
		self.robot_drive.front_left.skip_periodic_command = False
		print("[TEST] === EXITING TEST MODE - PID re-enabled ===\n")
