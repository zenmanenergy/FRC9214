# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
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
		
		# Reset startup flag so diagnostic prints on first cycle
		self.robot_drive.rear_left.startup_printed = False
		
		# Do NOT enable debug by default - it spams console
		# self.robot_drive.set_debug_mode(True)
		
		# Center all wheels on enable
		center_state = SwerveModuleState(0, Rotation2d(0))
		self.robot_drive.front_left.set_desired_state(center_state)
		self.robot_drive.front_right.set_desired_state(center_state)
		self.robot_drive.rear_left.set_desired_state(center_state)
		self.robot_drive.rear_right.set_desired_state(center_state)
		
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
			if not self.robot_drive.rear_left.autotune_active:
				center_state = SwerveModuleState(0, Rotation2d(0))
				self.robot_drive.rear_left.set_desired_state(center_state)
			
		except Exception as e:
			print(f"[ERROR] teleopPeriodic: {e}")
			import traceback
			traceback.print_exc()
			
		
	def teleopExit(self):
		"""Disable debug output when exiting teleop."""
		self.robot_drive.set_debug_mode(False)
		print("[ROBOT] === EXITING TELEOP MODE ===\n")

	def testInit(self):
		"""This function is called once when the robot enters test mode."""
		pass

	def testPeriodic(self):
		"""This function is called periodically during test mode."""
		pass

	def testExit(self):
		pass
