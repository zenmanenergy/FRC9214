# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib
from wpilib import XboxController
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
		# Initialize subsystems
		self.robot_drive = DriveSubsystem()
		
		# Initialize driver controller
		self.driver_controller = XboxController(OIConstants.k_driver_controller_port)

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
		self.autonomous_command = self.robot_container.get_autonomous_command()

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
		if self.autonomous_command is not None:
			self.autonomous_command.cancel()

	def teleopPeriodic(self):
		"""This function is called periodically during operator control."""
		from wpimath.util import MathUtil
		
		# Drive control - left stick for translation, right stick for rotation
		x_speed = -MathUtil.apply_deadband(
			self.driver_controller.get_left_y(), 
			OIConstants.k_drive_deadband)
		y_speed = -MathUtil.apply_deadband(
			self.driver_controller.get_left_x(), 
			OIConstants.k_drive_deadband)
		rot = -MathUtil.apply_deadband(
			self.driver_controller.get_right_x(), 
			OIConstants.k_drive_deadband)
		
		self.robot_drive.drive(x_speed, y_speed, rot, True)
		
		# Button bindings
		if self.driver_controller.getRB():
			self.robot_drive.set_x()
		
		if self.driver_controller.getStartButton():
			self.robot_drive.zero_heading()

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
