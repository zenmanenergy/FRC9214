

	

import wpilib
import rev
from arm_calibration import ArmCalibration
from arm import Arm

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.calibration = ArmCalibration()

		elevator_motor = rev.SparkMax(10, rev.SparkMax.MotorType.kBrushless)
		shoulder_motor = rev.SparkMax(12, rev.SparkMax.MotorType.kBrushless)
		wrist_motor = rev.SparkMax(11, rev.SparkMax.MotorType.kBrushless)
		grabber_motor = rev.SparkMax(13, rev.SparkMax.MotorType.kBrushless)

		self.ArmJoystick = wpilib.Joystick(0)  # Using a standard joystick
		self.arm = Arm(wpilib, self.ArmJoystick, elevator_motor, shoulder_motor, wrist_motor, grabber_motor)







		######################################################################
		### HOW TO USE:
		### Set these values to True/False to decide which joint to calibrate.
		### follow instructions in printout


		# FLAGS: Set these to True to calibrate a specific joint
		self.calibrate_elevator = False
		self.calibrate_shoulder = False
		self.calibrate_wrist = True
		

	def JoyStickPeriodic(self):
		self.LeftThumbUPDOWN=self.ArmJoystick.getRawAxis(1)*-1  #reverse the direction, so up is positive
		if abs(self.LeftThumbUPDOWN) < 0.05:
			self.LeftThumbUPDOWN=0
		self.RightThumbUPDOWN=self.ArmJoystick.getRawAxis(5)*-1
		if abs(self.RightThumbUPDOWN) < 0.05:
			self.RightThumbUPDOWN=0
		self.RightThumbLEFTRIGHT=self.ArmJoystick.getRawAxis(4)
		if abs(self.RightThumbLEFTRIGHT) < 0.05:
			self.RightThumbLEFTRIGHT=0
		LeftTrigger=self.ArmJoystick.getRawAxis(2)*-1
		if abs(LeftTrigger) < 0.05:
			LeftTrigger=0
		RightTrigger=self.ArmJoystick.getRawAxis(3)
		if abs(RightTrigger) < 0.05:
			RightTrigger=0
		self.LeftORRightTrigger=0
		if LeftTrigger <0.1:
			self.LeftORRightTrigger=LeftTrigger
		if RightTrigger>0.1:
			self.LeftORRightTrigger=RightTrigger
			

		# print(self.LeftThumbUPDOWN,self.RightThumbUPDOWN,self.RightThumbLEFTRIGHT,LeftTrigger,RightTrigger,self.LeftORRightTrigger)
		self.StartButton = self.ArmJoystick.getRawButton(8)  # Start button
		self.AButton = self.ArmJoystick.getRawButton(1)  # A button
		self.LBButton = self.ArmJoystick.getRawButton(5)  # LB button

		self.YButton = self.ArmJoystick.getRawButton(4)
		self.XButton = self.ArmJoystick.getRawButton(3)
		self.BButton = self.ArmJoystick.getRawButton(2)

	def teleopInit(self):
		calibration_targets = []
		if self.calibrate_elevator:
			calibration_targets.append("ELEVATOR")
		if self.calibrate_shoulder:
			calibration_targets.append("SHOULDER")
		if self.calibrate_wrist:
			calibration_targets.append("WRIST")

		if calibration_targets:
			print(f"[CALIBRATION] Press Start to calibrate: {', '.join(calibration_targets)}")

	def verify_encoders(self, arm):
		"""Prints current encoder values for verification when RB is pressed."""
		self.RBButton = self.ArmJoystick.getRawButton(6)
		if self.RBButton:
			print("\n[VERIFICATION] Current Encoder Values:")
			print(f"  Elevator Encoder: {arm.elevator_encoder.getPosition():.6f}")
			print(f"  Shoulder Encoder: {arm.shoulder_encoder.getPosition():.6f}")
			print(f"  Wrist Encoder: {arm.wrist_encoder.getPosition():.6f}")
			print("[VERIFICATION] Press RB anytime to check encoder values.\n")
	

	def teleopPeriodic(self):
		self.JoyStickPeriodic()

		self.verify_encoders(self.arm)
		# Control joints using thumbsticks (always available)
		self.arm.control_elevator(self.LeftThumbUPDOWN)
		self.arm.control_shoulder(self.RightThumbUPDOWN)
		self.arm.control_wrist(self.RightThumbLEFTRIGHT)

		

		# Start Calibration if Start Button is Pressed
		if self.calibration.state == "idle":
			if self.calibrate_elevator:
				print("[CALIBRATION] Starting Elevator Calibration")
				self.calibration.start_elevator_calibration(self.arm)
			elif self.calibrate_shoulder:
				print("[CALIBRATION] Starting Shoulder Calibration")
				self.calibration.start_shoulder_calibration(self.arm)
			elif self.calibrate_wrist:
				print("[CALIBRATION] Starting Wrist Calibration")
				self.calibration.start_wrist_calibration(self.arm)

		# Update calibration process (non-blocking)
		self.calibration.update(self.ArmJoystick, self.arm)
