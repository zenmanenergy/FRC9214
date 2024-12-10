import wpilib
import ctre
import rev
from ctre import NeutralMode
from math import atan2, degrees, pi

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		# Joystick and motor setup
		self.DriveJoystick = wpilib.Joystick(0)
		self.LeftFrontMotor = ctre.WPI_TalonSRX(1)
		self.LeftRearMotor = ctre.WPI_TalonSRX(2)
		self.RightFrontMotor = ctre.WPI_TalonSRX(3)
		self.RightRearMotor = ctre.WPI_TalonSRX(4)

		self.LeftFrontMotor.setNeutralMode(NeutralMode.Brake)
		self.LeftRearMotor.setNeutralMode(NeutralMode.Brake)
		self.RightFrontMotor.setNeutralMode(NeutralMode.Brake)
		self.RightRearMotor.setNeutralMode(NeutralMode.Brake)

		# IMU initialization
		self.gyro = wpilib.ADXRS450_Gyro()
		self.gyro.reset()

		# Travel state
		self.target_heading = None
		self.rotating = False

	def teleopInit(self):
		# Reset gyro heading at the start of teleop
		self.gyro.reset()

	def teleopPeriodic(self):
		# Read joystick buttons
		self.DRIVE_BUTTON_X = self.DriveJoystick.getRawButton(3)  # X button
		self.DRIVE_BUTTON_A = self.DriveJoystick.getRawButton(1)  # A button
		self.DRIVE_BUTTON_B = self.DriveJoystick.getRawButton(2)  # B button
		self.DRIVE_BUTTON_Y = self.DriveJoystick.getRawButton(4)  # Y button

		# Handle rotation to absolute headings
		if self.DRIVE_BUTTON_X:
			self.target_heading = 270  # Face left
			self.startRotation()
		elif self.DRIVE_BUTTON_A:
			self.target_heading = 180  # Face backward
			self.startRotation()
		elif self.DRIVE_BUTTON_B:
			self.target_heading = 90  # Face right
			self.startRotation()
		elif self.DRIVE_BUTTON_Y:
			self.target_heading = 0  # Face forward
			self.startRotation()

		# Continue rotation if in progress
		if self.rotating:
			self.rotating = self.rotateToHeading()

	def startRotation(self):
		"""
		Initializes rotation by setting the rotating flag.
		"""
		self.rotating = True

	def rotateToHeading(self):
		"""
		Rotates the robot to the target heading.
		Returns True if still rotating, False if the target heading is reached.
		"""
		current_heading = self.normalizeAngle(self.gyro.getAngle())
		target_heading = self.target_heading
		error = self.normalizeAngle(target_heading - current_heading)

		# Print debug info
		print(f"Current: {current_heading:.2f}°, Target: {target_heading:.2f}°, Error: {error:.2f}°")

		# Rotation speed based on error
		if abs(error) > 2:  # Threshold to stop rotation
			speed = 0.4 if error > 0 else -0.4
			self.LeftFrontMotor.set(-speed)
			self.LeftRearMotor.set(-speed)
			self.RightFrontMotor.set(speed)
			self.RightRearMotor.set(speed)
			return True
		else:
			# Stop the motors
			self.LeftFrontMotor.set(0)
			self.LeftRearMotor.set(0)
			self.RightFrontMotor.set(0)
			self.RightRearMotor.set(0)
			return False

	def normalizeAngle(self, angle):
		"""
		Normalizes an angle to the range [0, 360).
		"""
		return angle % 360

if __name__ == "__main__":
	wpilib.run(MyRobot)
