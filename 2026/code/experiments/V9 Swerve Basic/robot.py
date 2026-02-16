import wpilib
import wpilib.drive
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

from drive import SwerveDrive, SwerveModule
from joystick_drive import DriverJoystick
from joystick_arm import ArmJoystick



# ==============================================================================
# MAIN ROBOT CLASS
# ==============================================================================


class Robot(wpilib.TimedRobot):
	"""Main robot container and teleop code."""

	def robotInit(self):
		"""Initialize robot and subsystems."""
		try:
			print("\n===== ROBOT INIT STARTING =====")
			
			# Initialize joysticks
			print("[INIT] Creating joysticks...")
			self.driver = DriverJoystick()
			self.operator = ArmJoystick()
			print("[INIT] [OK] Joysticks created")

			# Initialize swerve as None in case creation fails
			self.swerve = None

			# Create swerve modules with REV CANSparkMax motors
			# Adjust CAN IDs and encoder offsets as needed
			print("[INIT] Creating swerve modules...")
			try:
				print("[INIT]   Creating FL module (CAN 2, 3)...")
				self.front_left = SwerveModule(drive_motor_id=2, turn_motor_id=3, encoder_offset=0.0)
				print("[INIT]   [OK] FL created")
				
				print("[INIT]   Creating FR module (CAN 8, 9)...")
				self.front_right = SwerveModule(drive_motor_id=8, turn_motor_id=9, encoder_offset=0.0)
				print("[INIT]   [OK] FR created")
				
				print("[INIT]   Creating RL module (CAN 4, 5)...")
				self.rear_left = SwerveModule(drive_motor_id=4, turn_motor_id=5, encoder_offset=0.0)
				print("[INIT]   [OK] RL created")
				
				print("[INIT]   Creating RR module (CAN 6, 7)...")
				self.rear_right = SwerveModule(drive_motor_id=6, turn_motor_id=7, encoder_offset=0.0)
				print("[INIT]   [OK] RR created")
				
				print("[INIT] SUCCESS: All swerve modules created")
			except Exception as e:
				print("[INIT] ERROR creating modules: %s" % e)
				import traceback
				traceback.print_exc()
				print("[INIT] Returning early due to module creation failure")
				return

			# Create swerve drive
			print("[INIT] Creating SwerveDrive...")
			try:
				self.swerve = SwerveDrive(
					self.front_left,
					self.front_right,
					self.rear_left,
					self.rear_right,
					track_width_m=0.5,
					wheelbase_m=0.5,
				)
				print("[INIT] SUCCESS: SwerveDrive created")
			except Exception as e:
				print("[INIT] ERROR creating SwerveDrive: %s" % e)
				import traceback
				traceback.print_exc()
				self.swerve = None
			
			print("===== ROBOT INIT COMPLETE =====\n")
		except Exception as e:
			print("[INIT] FATAL ERROR in robotInit: %s" % e)
			import traceback
			traceback.print_exc()

		# Optional: Initialize gyro for field-relative drive
		# self.gyro = wpilib.ADIS16448_IMU()

		# Drive mode flags
		self.slow_mode = False

	def teleopInit(self):
		"""Called when entering teleop mode - reset everything."""
		print("[ROBOT] Teleop mode started - resetting swerve")
		if self.swerve is not None:
			self.swerve.stop()
			# Reset module states and stop each module
			try:
				for module in [self.front_left, self.front_right, self.rear_left, self.rear_right]:
					module.current_state = SwerveModuleState()
					module.stop()
			except Exception:
				pass
		return super().teleopInit()

	def teleopPeriodic(self):
		"""Handle teleop control inputs."""
		if self.swerve is None:
			return

		# Get joystick inputs (deadband already applied)
		forward = self.driver.get_forward()
		strafe = self.driver.get_strafe()
		rotation = self.driver.get_rotation()
		
		# Only print when there's actual motion
		if abs(forward) > 0.15 or abs(strafe) > 0.15 or abs(rotation) > 0.15:
			print("INPUTS - Forward: %.3f Strafe: %.3f Rotation: %.3f" % (forward, strafe, rotation))

		# Apply slow mode multiplier
		if self.driver.get_slow_mode_button():
			self.slow_mode = True
		else:
			self.slow_mode = False

		if self.slow_mode:
			forward *= 0.3

		# Stop button
		if self.driver.get_stop_button():
			self.swerve.stop()
		else:
			self.swerve.drive(forward, strafe, rotation, field_relative=False)

	def disabledInit(self):
		"""Called when robot enters disabled state."""
		print("[ROBOT] Disabled state entered")
		if self.swerve is not None:
			self.swerve.stop()
			# Reset module states so angle tracking starts fresh on re-enable
			try:
				for module in [self.front_left, self.front_right, self.rear_left, self.rear_right]:
					module.current_state = SwerveModuleState()
				self.front_left.stop()
				self.front_right.stop()
				self.rear_left.stop()
				self.rear_right.stop()
			except Exception:
				pass

	def disabledPeriodic(self):
		"""Called periodically while robot is disabled."""
		pass

	# ========================================================================
	# UTILITY FUNCTIONS
	# ========================================================================

	def _print_debug_info(self, forward: float, strafe: float, rotation: float):
		"""Print debug information to console and SmartDashboard."""
		# Joystick inputs
		wpilib.SmartDashboard.putNumber("Joystick Forward", forward)
		wpilib.SmartDashboard.putNumber("Joystick Strafe", strafe)
		wpilib.SmartDashboard.putNumber("Joystick Rotation", rotation)
		
		# Module angles
		angles = self.swerve.get_module_angles()
		wpilib.SmartDashboard.putNumber("FL Angle", angles[0].degrees())
		wpilib.SmartDashboard.putNumber("FR Angle", angles[1].degrees())
		wpilib.SmartDashboard.putNumber("RL Angle", angles[2].degrees())
		wpilib.SmartDashboard.putNumber("RR Angle", angles[3].degrees())
		
		# Slow mode and stop button status
		wpilib.SmartDashboard.putBoolean("Slow Mode", self.slow_mode)
		wpilib.SmartDashboard.putBoolean("Stop Button", self.driver.get_stop_button())
		
		# Only print if there's motion
		if abs(forward) > 0.01 or abs(strafe) > 0.01 or abs(rotation) > 0.01:
			print("JOYSTICK - Forward: %.2f, Strafe: %.2f, Rotation: %.2f" % (forward, strafe, rotation))


# ==============================================================================
# ENTRY POINT
# ==============================================================================

if __name__ == "__main__":
	wpilib.run(Robot)
