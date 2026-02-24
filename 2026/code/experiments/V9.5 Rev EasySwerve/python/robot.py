# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
from wpilib import Joystick
from constants import OIConstants
from subsystems.drive_subsystem import DriveSubsystem


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
		pass

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
		print("[ROBOT] teleopInit() - entering teleop mode")
		if self.autonomous_command is not None:
			self.autonomous_command.cancel()
		
		# Print motor diagnostics
		print("\n[ROBOT] === MOTOR DIAGNOSTICS ===")
		self.robot_drive.front_left.print_diagnostics()
		self.robot_drive.front_right.print_diagnostics()
		self.robot_drive.rear_left.print_diagnostics()
		self.robot_drive.rear_right.print_diagnostics()
		
		# CENTER ALL WHEELS - Command all turn motors to 0 radians
		print("\n[ROBOT] Commanding all wheels to center (0 radians)...")
		from wpimath.geometry import Rotation2d
		from wpimath.kinematics import SwerveModuleState
		center_state = SwerveModuleState(0, Rotation2d(0))
		
		self.robot_drive.front_left.set_desired_state(center_state)
		self.robot_drive.front_right.set_desired_state(center_state)
		self.robot_drive.rear_left.set_desired_state(center_state)
		self.robot_drive.rear_right.set_desired_state(center_state)
		
		print("[ROBOT] teleopInit() complete")

	def teleopPeriodic(self):
		"""This function is called periodically during operator control."""
		
		try:
			# Drive control - single joystick with dual thumb sticks
			def apply_deadband(value, deadband):
				return value if abs(value) > deadband else 0.0
			
			# Joystick axis mapping for Xbox controller:
			# Axis 0: Left thumb left/right (X)
			# Axis 1: Left thumb up/down (Y)
			# Axis 3: Right thumb left/right (X)
			# Axis 4: Right thumb up/down (Y)
			raw_y = self.driver_joystick.getRawAxis(1)  # Left thumb up/down: forward/back
			raw_x = self.driver_joystick.getRawAxis(0)  # Left thumb left/right: strafe
			raw_rot = self.driver_joystick.getRawAxis(3)  # Right thumb left/right: rotation
			
			x_speed = -apply_deadband(raw_y, OIConstants.k_drive_deadband)
			y_speed = -apply_deadband(raw_x, OIConstants.k_drive_deadband)
			rot = -apply_deadband(raw_rot, OIConstants.k_drive_deadband)
			
			self.robot_drive.drive(x_speed, y_speed, rot, True)
			
			# Button bindings
			if self.driver_joystick.getRawButton(1):  # A button
				self.robot_drive.spin_turn_motors(0.5)
			elif self.driver_joystick.getRawButton(2):  # B button
				self.robot_drive.spin_turn_motors(-0.5)
			elif self.driver_joystick.getRawButton(3):  # X button
				self.robot_drive.spin_turn_motors(0)
			elif self.driver_joystick.getRawButton(4):  # Y button - test without encoder
				self.robot_drive.test_turn_motors_no_encoder(0.5)
			
			if self.driver_joystick.getRawButton(5):
				self.robot_drive.set_x()
			
			if self.driver_joystick.getRawButton(6):
				self.robot_drive.zero_heading()
		except Exception as e:
			print(f"[ERROR] teleopPeriodic: {e}")
			import traceback
			traceback.print_exc()

	def teleopExit(self):
		pass

	def testInit(self):
		"""This function is called once when the robot enters test mode."""
		pass

	def testPeriodic(self):
		"""This function is called periodically during test mode."""
		pass

	def testExit(self):
		pass
