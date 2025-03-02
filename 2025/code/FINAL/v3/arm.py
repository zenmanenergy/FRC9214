
class Arm:
	
	def __init__(self, ArmJoystick, elevator_motor, shoulder_motor, wrist_motor, grabber_motor):
		self.ArmJoystick=ArmJoystick
		self.elevator_motor=elevator_motor
		self.shoulder_motor=shoulder_motor
		self.wrist_motor=wrist_motor
		self.grabber_motor=grabber_motor

		self.elevator_encoder = self.elevator_motor.getEncoder()
		self.shoulder_encoder = self.shoulder_motor.getEncoder()
		self.wrist_encoder = self.wrist_motor.getEncoder()
		self.grabber_encoder = self.grabber_motor.getEncoder()

		self.elevatorUpSpeedFactor = 0.4
		self.elevatorDownSpeedFactor = 0.4
		self.elevatorBreakSpeed = 0.05
		self.elevator_min_height = 0
		self.elevator_max_height = 60


		self.shoulderUpSpeedFactor = 0.3
		self.shoulderDownSpeedFactor = 0.05
		self.shoulderBreakSpeed = 0.12
		self.minShoulderBreakSpeed = 0.06
		self.maxShoulderBreakSpeed = 0.06
		self.shoulder_min_angle = 10
		self.shoulder_max_angle = 140

		self.wristUpSpeedFactor = 0.2
		self.wristDownSpeedFactor = 0.25
		self.wristBreakSpeed = 0.03
		self.wrist_min_angle = -100
		self.wrist_max_angle = 220

		self.grabberInSpeedFactor = 0.5
		self.grabberOutSpeedFactor = 0.5
		self.grabberBreakSpeed = 0.0

		
		self.shoulder_deg_per_tick = 6.395935
		self.shoulder_zero_offset = -4.738090


		self.elevator_cm_per_tick = 1.473699
		self.elevator_zero_offset = -1.214286

		self.wrist_deg_per_tick = -3.716837
		self.wrist_zero_offset = 0.000000

		self.MAX_CURRENT = 60

		self.real_elevator_position = 0.0
		self.real_arm_angle = 0.0
		self.real_wrist_angle = 0.0
		self.real_grabber_angle = 0.0

		self.prev_elevator_position = 0
		self.prev_arm_angle = 0
		self.prev_wrist_angle = 0
		self.prev_grabber_angle = 0

		self.LeftThumbUPDOWN = 0
		self.RightThumbUPDOWN = 0
		self.RightThumbLEFTRIGHT = 0
		self.LeftORRightTrigger = 0


		self.presets = {
			"A":    {"elevator": 10, "shoulder": 30, "wrist": 50},
			"Y":    {"elevator": 20, "shoulder": 60, "wrist": 90},
			"X":    {"elevator": 5,  "shoulder": 45, "wrist": 70},
			"B":    {"elevator": 15, "shoulder": 90, "wrist": 110},

			"LB+A": {"elevator": 25, "shoulder": 100, "wrist": 130},
			"LB+Y": {"elevator": 30, "shoulder": 110, "wrist": 140},
			"LB+X": {"elevator": 8,  "shoulder": 50, "wrist": 60},
			"LB+B": {"elevator": 12, "shoulder": 75, "wrist": 95}
		}

	def reset(self):
		self.resetEncoders()
	
	def resetEncoders(self):
		self.elevator_encoder.setPosition(0)
		self.shoulder_encoder.setPosition(0)
		self.wrist_encoder.setPosition(0)
		self.grabber_encoder.setPosition(0)

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
		if abs(self.LeftORRightTrigger) < 0.05:
			self.LeftORRightTrigger=0


		self.AButton = self.ArmJoystick.getRawButton(1)  # A button
		self.YButton = self.ArmJoystick.getRawButton(4)
		self.XButton = self.ArmJoystick.getRawButton(3)
		self.BButton = self.ArmJoystick.getRawButton(2)

		self.LBButton = self.ArmJoystick.getRawButton(5)  # LB button


	def periodic(self, debug):
		self.JoyStickPeriodic()
		self.real_elevator_position = (self.elevator_encoder.getPosition() - self.elevator_zero_offset) * self.elevator_cm_per_tick
		self.real_arm_angle = (self.shoulder_encoder.getPosition() - self.shoulder_zero_offset) * self.shoulder_deg_per_tick
		self.real_wrist_angle = (self.wrist_encoder.getPosition() - self.wrist_zero_offset) * self.wrist_deg_per_tick
		self.real_grabber_angle = self.grabber_encoder.getPosition()


		if debug and (
			self.real_elevator_position != self.prev_elevator_position or
			self.real_arm_angle != self.prev_arm_angle or
			self.real_wrist_angle != self.prev_wrist_angle or
			self.real_grabber_angle != self.prev_grabber_angle
		):
			print(f"REAL -> Elevator: {self.real_elevator_position:.2f}, Arm: {self.real_arm_angle:.2f} degrees, "
				f"Wrist: {self.real_wrist_angle:.2f} degrees, Grabber: {self.real_grabber_angle:.2f}")
			

		lb_pressed = self.LBButton
		buttons = {
			"A": self.AButton,
			"Y": self.YButton,
			"X": self.XButton,
			"B": self.BButton
		}

		# Check for presets
		for name, pressed in buttons.items():
			if pressed:
				preset_name = f"LB+{name}" if lb_pressed else name
				if preset_name in self.presets:
					self.apply_preset(self.presets[preset_name])
					break  # Prevent multiple presets being set at once

		if self.active_preset:
			self.move_to_preset()
		else:
			# Normal manual control when no preset is active
			self.control_elevator(self.LeftThumbUPDOWN)
			self.control_shoulder(self.RightThumbUPDOWN)
			self.control_wrist(self.RightThumbLEFTRIGHT)
			self.control_grabber(self.LeftORRightTrigger)
	
	def move_to_preset(self):
		"""Move arm components toward the preset positions and stop when reached."""
		target_elevator = self.active_preset["elevator"]
		target_shoulder = self.active_preset["shoulder"]
		target_wrist = self.active_preset["wrist"]

		# Compute speed to move toward the preset smoothly
		elevator_speed = (target_elevator - self.real_elevator_position) * 0.1
		shoulder_speed = (target_shoulder - self.real_arm_angle) * 0.1
		wrist_speed = (target_wrist - self.real_wrist_angle) * 0.1

		# Apply speed limits
		elevator_speed = max(min(elevator_speed, 0.5), -0.5)
		shoulder_speed = max(min(shoulder_speed, 0.3), -0.3)
		wrist_speed = max(min(wrist_speed, 0.2), -0.2)

		# Stop movement if within target range
		if abs(self.real_elevator_position - target_elevator) < 1:
			elevator_speed = 0
		if abs(self.real_arm_angle - target_shoulder) < 2:
			shoulder_speed = 0
		if abs(self.real_wrist_angle - target_wrist) < 2:
			wrist_speed = 0

		# Move motors
		self.control_elevator(elevator_speed)
		self.control_shoulder(shoulder_speed)
		self.control_wrist(wrist_speed)

		# If all joints are at the target, clear the active preset (prevents movement drift)
		if elevator_speed == 0 and shoulder_speed == 0 and wrist_speed == 0:
			self.active_preset = None



	
	#######################################################
	#elevator - GPT please keep this comment here 
	#######################################################
	
	def limit_elevator(self, elevator_speed):
		"""Ensures elevator does not exceed its movement limits based on joystick direction."""
		if elevator_speed > 0 and self.real_elevator_position >= self.elevator_max_height:
			return False  # Stop moving upward
		if elevator_speed < 0 and self.real_elevator_position <= self.elevator_min_height:
			return False  # Stop moving downward
		return True	

	def control_elevator(self, elevator_speed):
		"""Controls the elevator motor with braking and limit checks."""
		if not self.limit_elevator(elevator_speed):  # Prevent movement beyond limits
			elevator_speed = 0  # Prevent movement beyond limits
		elif abs(elevator_speed) < 0.01: #Break the motor if the joystick is neutral
			elevator_speed = self.elevatorBreakSpeed
		else:
			elevator_speed *= self.elevatorUpSpeedFactor if elevator_speed > 0 else self.elevatorDownSpeedFactor

		self.elevator_motor.set(elevator_speed)

	
	
	
	
	#######################################################
	#shoulder - GPT please keep this comment here 
	#######################################################
	
	def limit_shoulder(self, shoulder_speed):
		"""Ensures shoulder does not exceed its movement limits based on joystick direction."""
		if shoulder_speed > 0 and self.real_arm_angle >= self.shoulder_max_angle:
			return False  # Stop moving upward
		if shoulder_speed < 0 and self.real_arm_angle <= self.shoulder_min_angle:
			return False  # Stop moving downward
		return True
	
	def control_shoulder(self, shoulder_speed):
		"""Controls the shoulder motor with braking and limit checks."""
		
		if self.real_arm_angle < 70 or self.real_arm_angle > 120:
			self.shoulderBreakSpeed = self.minShoulderBreakSpeed
		else:
			self.shoulderBreakSpeed = self.maxShoulderBreakSpeed

		if not self.limit_shoulder(shoulder_speed):  # Prevent movement beyond limits
			shoulder_speed = 0 
		elif abs(shoulder_speed) < 0.01:  #Break the motor if the joystick is neutral
			shoulder_speed = self.shoulderBreakSpeed
		elif shoulder_speed > 0:
			shoulder_speed *= self.shoulderUpSpeedFactor
		elif shoulder_speed < 0:
			shoulder_speed *= self.shoulderDownSpeedFactor

		self.shoulder_motor.set(shoulder_speed) #Turn the motor on/off



	#######################################################
	#wrist - GPT please keep this comment here 
	#######################################################

	def limit_wrist(self, wrist_speed):

		# If the wrist is BELOW the minimum limit, allow ONLY upward movement
		if self.real_wrist_angle < self.wrist_min_angle and wrist_speed > 0:
			return False  # Block downward movement
		if self.real_wrist_angle < self.wrist_min_angle and wrist_speed < 0:
			return True   # Allow upward movement

		# If the wrist is ABOVE the maximum limit, allow ONLY downward movement
		if self.real_wrist_angle > self.wrist_max_angle and wrist_speed < 0:
			return False  # Block upward movement
		if self.real_wrist_angle > self.wrist_max_angle and wrist_speed > 0:
			return True   # Allow downward movement

		# Otherwise, movement is unrestricted
		return True
	
	def control_wrist(self, wrist_speed):
		"""Controls the wrist motor with braking and limit checks."""
		if not self.limit_wrist(wrist_speed):
			wrist_speed = 0  # Prevent movement beyond limits
		
		elif abs(wrist_speed) < 0.01: #Break the motor if the joystick is neutral
			wrist_speed = self.wristBreakSpeed
		
		elif wrist_speed > 0: #Move up
			wrist_speed *= self.wristUpSpeedFactor

		elif wrist_speed < 0: #Move down
			wrist_speed *= self.wristDownSpeedFactor


		self.wrist_motor.set(wrist_speed) #Turn the motor on/off
	

	
	#######################################################
	#grabber - GPT please keep this comment here 
	#######################################################

	def control_grabber(self, grabber_speed):
		"""Controls the grabber motor with braking."""
		grabber_speed *= self.grabberInSpeedFactor if grabber_speed > 0 else self.grabberOutSpeedFactor
		if abs(grabber_speed) < 0.01: #Break the motor if the joystick is neutral
			grabber_speed = self.grabberBreakSpeed

		self.grabber_motor.set(grabber_speed) #Turn the motor on/off