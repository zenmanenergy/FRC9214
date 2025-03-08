from wpimath.controller import PIDController

class Arm:
	
	def __init__(self, wpilib, ArmJoystick, elevator_motor, shoulder_motor, wrist_motor, grabber_motor):
		self.autoPreset = None
		self.isAuto = False
		
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
		self.elevatorPresetFactor = 0.1
		self.elevatorBreakSpeed = 0.05
		self.elevator_min_height = 43.6
		self.elevator_max_height = 104


		self.shoulderUpSpeedFactor = 0.3
		self.shoulderDownSpeedFactor = 0.3
		self.shoulderBreakSpeed = 0.12
		self.shoulderPresetFactor = 0.1
		self.minShoulderBreakSpeed = 0.053
		self.maxShoulderBreakSpeed = 0.053
		self.shoulder_min_angle = 10
		self.shoulder_max_angle = 175

		self.wristUpSpeedFactor = 0.2
		self.wristDownSpeedFactor = 0.25
		self.wristPresetFactor = 0.1
		self.wristBreakSpeed = 0.05
		self.wrist_min_angle = -160
		self.wrist_max_angle = 88

		self.grabberInSpeedFactor = 0.5
		self.grabberOutSpeedFactor = 0.5
		self.grabberBreakSpeed = 0.0

		
		# self.shoulder_deg_per_tick = 6.395935
		# self.shoulder_zero_offset = -4.738090

		# self.elevator_cm_per_tick = 1.473699
		# self.elevator_zero_offset = -1.214286

		# self.wrist_deg_per_tick = -3.716837
		# self.wrist_zero_offset = 0.000000


		self.shoulder_deg_per_tick = 6.822438 
		self.shoulder_zero_offset = -63.189617 

		
		self.elevator_cm_per_tick = 1.430248 
		self.elevator_zero_offset = 13.109511

 

		self.wrist_deg_per_tick = -3.768716
		self.wrist_zero_offset = -0.214286


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
		
		self.active_preset = None
		self.prev_wrist_speed=None
		self.prev_wrist_angle=None

		self.presets = {
			"B":    {"elevator":44, "shoulder": 8, "wrist": 41},  #loading coral
			"A":    {"elevator": 48, "shoulder": 5, "wrist": -117}, #scoring on L2
			"X":    {"elevator":98,  "shoulder": 32, "wrist": -53}, #scoring on L3
			"Y":    {"elevator": 110, "shoulder": 57, "wrist": 55}, #scoring on L4

			"LB+A": {"elevator": 69, "shoulder": 11, "wrist": -32},
			"LB+Y": {"elevator": 30, "shoulder": 110, "wrist": 140},
			"LB+X": {"elevator": 8,  "shoulder": 50, "wrist": 60},
			"LB+B": {"elevator": 12, "shoulder": 75, "wrist": 95},

			"EUp": {"elevator": 70, "shoulder": -75, "wrist": -47}
		}

		# Elevator PID Controller
		self.elevator_kP = 0.05 # Proportional gain (increase if it's too slow)
		self.elevator_kI = 0.00   # Integral gain (increase if there's steady-state error)
		self.elevator_kD = 0.00   # Derivative gain (increase if there's oscillation)

		
		self.elevator_pid = PIDController(
			self.elevator_kP, self.elevator_kI, self.elevator_kD
		)
		self.elevator_pid.setTolerance(0.5)  # Allowable error in position

		# Shoulder PID Controller
		self.shoulder_kP = 0.015
		  # Adjust this for speed
		self.shoulder_kI = 0.00  # Only increase if it struggles to reach target
		self.shoulder_kD = 0.001  # Helps smooth stopping

		self.shoulder_pid = PIDController(
			self.shoulder_kP, self.shoulder_kI, self.shoulder_kD
		)
		self.shoulder_pid.setTolerance(1.0)  # Stops adjustments within 1 degree

		# Wrist PID Controller
		self.wrist_kP = 0.015  # Adjust this for speed
		self.wrist_kI = 0.00  # Only increase if it struggles to reach target
		self.wrist_kD = 0.001  # Helps smooth stopping

		self.wrist_pid = PIDController(
			self.wrist_kP, self.wrist_kI, self.wrist_kD
		)
		self.wrist_pid.setTolerance(1.0)  # Stops adjustments within 1.5 degrees


	def reset(self):
		self.resetEncoders()
		self.active_preset = None
	
	def resetEncoders(self):
		self.set_joint_position("elevator", 51)
		self.set_joint_position("shoulder", -75)
		self.set_joint_position("wrist", -47)
		
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
		self.RBButton=self.ArmJoystick.getRawButton(6)
		self.StartButton = self.ArmJoystick.getRawButton(8)  # Start button

		# 🔥 **Manual Wrist Reset ONLY when Start button is pressed**
		if self.StartButton:
			self.set_joint_position("wrist", 0)  # Set wrist to 0 degrees

		if self.LBButton:
			self.set_joint_position("shoulder", 0)  # Set shoulder to 360 degrees

	def set_joint_position(self, joint, target_value):
		"""
		Sets the encoder position for a given joint to match a desired angle (degrees) or height (cm).
		
		:param joint: The joint to modify ("wrist", "shoulder", or "elevator").
		:param target_value: The desired position (degrees for wrist/shoulder, cm for elevator).
		"""
		# Determine which joint is being set and get the correct conversion factors
		if joint == "wrist":
			unit_per_tick = self.wrist_deg_per_tick
			zero_offset = self.wrist_zero_offset
			encoder = self.wrist_encoder
		elif joint == "shoulder":
			unit_per_tick = self.shoulder_deg_per_tick
			zero_offset = self.shoulder_zero_offset
			encoder = self.shoulder_encoder
		elif joint == "elevator":
			unit_per_tick = self.elevator_cm_per_tick  # Uses cm instead of degrees
			zero_offset = self.elevator_zero_offset
			encoder = self.elevator_encoder
		else:
			print(f"[ERROR] Unknown joint '{joint}' in set_joint_position()")
			return

		# Convert the desired value (degrees or cm) into an encoder position
		new_encoder_value = (target_value / unit_per_tick) + zero_offset

		# Set the encoder position
		encoder.setPosition(new_encoder_value)
		print(f"[INFO] {joint.upper()} set to {target_value} ({'cm' if joint == 'elevator' else 'degrees'}) "
			  f"(Encoder Value: {new_encoder_value:.6f})")


	def periodic(self):
		preset_name=None
		self.JoyStickPeriodic()
		self.real_elevator_position = (self.elevator_encoder.getPosition() - self.elevator_zero_offset) * self.elevator_cm_per_tick
		self.real_arm_angle = (self.shoulder_encoder.getPosition() - self.shoulder_zero_offset) * self.shoulder_deg_per_tick
		self.real_wrist_angle = (self.wrist_encoder.getPosition() - self.wrist_zero_offset) * self.wrist_deg_per_tick
		self.real_grabber_angle = self.grabber_encoder.getPosition()

		if self.RBButton:
			print(f"REAL -> Elevator: {self.real_elevator_position:.2f}, Shoulder: {self.real_arm_angle:.2f} degrees, "
				f"Wrist: {self.real_wrist_angle:.2f} degrees, Grabber: {self.real_grabber_angle:.2f}"
				f" preset: {self.active_preset} preset_name: {preset_name}"
				)
			
		
		# Read button states
		lb_pressed = self.LBButton
		buttons = {
			"A": self.AButton,
			"Y": self.YButton,
			"X": self.XButton,
			"B": self.BButton
		}

		button_pressed = False
		for name, pressed in buttons.items():
			if pressed:  # If button is held, move to the preset
				button_pressed = True
				preset_name = f"LB+{name}" if lb_pressed else name
				if preset_name in self.presets:
					self.active_preset = self.presets[preset_name]  # Set active preset
				break  # Only track one preset at a time

		# If no button is pressed, reset active_preset
		if not button_pressed:
			self.active_preset = None  # Reset when the button is released
		
		if self.autoPreset:
			self.active_preset = self.presets[self.autoPreset]


		if self.active_preset:
			self.move_to_preset()

		# Normal manual control (only if no preset is active)
		else:
			
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
		# elevator_speed = (target_elevator - self.real_elevator_position) * self.elevatorPresetFactor
		# shoulder_speed = (target_shoulder - self.real_arm_angle) * self.shoulderPresetFactor
		# wrist_speed = (target_wrist - self.real_wrist_angle) * self.wristPresetFactor

		elevator_speed = self.elevator_pid.calculate(self.real_elevator_position, target_elevator) * self.elevatorUpSpeedFactor
		shoulder_speed = self.shoulder_pid.calculate(self.real_arm_angle, target_shoulder) * self.shoulderUpSpeedFactor
		wrist_speed = -self.wrist_pid.calculate(self.real_wrist_angle, target_wrist) * self.wristUpSpeedFactor

		# Apply speed limits

		elevator_speed = max(min(elevator_speed, 0.4), -0.4)
		shoulder_speed = max(min(shoulder_speed, 0.3), -0.3)
		wrist_speed = max(min(wrist_speed, 0.2), -0.2)

		if self.elevator_pid.atSetpoint():
			elevator_speed = 0
		if self.elevator_pid.atSetpoint():
			elevator_speed = 0
		if self.shoulder_pid.atSetpoint():
			shoulder_speed = 0
		if self.wrist_pid.atSetpoint():
			wrist_speed = 0
		

		# Stop movement if within target range
		if abs(self.real_elevator_position - target_elevator) < .5:
			elevator_speed = 0
		if abs(self.real_arm_angle - target_shoulder) < .5:
			shoulder_speed = 0
		if abs(self.real_wrist_angle - target_wrist) < .5:
			wrist_speed = 0

		# Move motors
		self.elevator_motor.set(elevator_speed)
		self.shoulder_motor.set(shoulder_speed)
		self.wrist_motor.set(wrist_speed)


		# If all joints are at the target, clear the active preset (prevents movement drift)
		if elevator_speed == 0 and shoulder_speed == 0 and wrist_speed == 0:
			self.active_preset = None

		# if (
		# 	self.real_elevator_position != self.prev_elevator_position or
		# 	self.real_arm_angle != self.prev_arm_angle or
		# 	self.real_wrist_angle != self.prev_wrist_angle
		# ):
		# 	print(f"\n--- Moving to Preset ---")
		# 	print(f"Target Positions -> Elevator: {target_elevator}, Shoulder: {target_shoulder}, Wrist: {target_wrist}")
		# 	print(f"Current Positions -> Elevator: {self.real_elevator_position:.2f}, Shoulder: {self.real_arm_angle:.2f}, Wrist: {self.real_wrist_angle:.2f}")
		# 	print(f"Speeds -> Elevator: {elevator_speed:.2f}, Shoulder: {shoulder_speed:.2f}, Wrist: {wrist_speed:.2f}")

		

		# Update previous values to track changes
		self.prev_elevator_position = self.real_elevator_position
		self.prev_arm_angle = self.real_arm_angle
		self.prev_wrist_angle = self.real_wrist_angle


	
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

		if self.real_elevator_position < 44:
			breakspeed=0
		else:
			breakspeed=self.elevatorBreakSpeed

		if not self.limit_elevator(elevator_speed):  # Prevent movement beyond limits
			elevator_speed = 0  # Prevent movement beyond limits
		elif abs(elevator_speed) < self.elevatorBreakSpeed: #Break the motor if the joystick is neutral
			elevator_speed = breakspeed
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
		if self.real_arm_angle > 180 and shoulder_speed < 0:
			return True
		

		return True
	
	
	def control_shoulder(self, shoulder_speed):
		"""Controls the shoulder motor with braking and limit checks, and adjusts the wrist if needed."""
		
		
		if self.real_arm_angle < -30 or self.real_arm_angle > 40:
			breakSpeed = self.minShoulderBreakSpeed
		else:
			breakSpeed = self.maxShoulderBreakSpeed

		if self.real_arm_angle < -50:
			breakSpeed = 0

		# Prevent exceeding shoulder limits
		if not self.limit_shoulder(shoulder_speed):
			shoulder_speed = 0  # Stop movement beyond limits

		elif abs(shoulder_speed) < self.maxShoulderBreakSpeed:  # Apply braking when joystick is neutral
			shoulder_speed = breakSpeed

		elif shoulder_speed > 0 and self.real_arm_angle <= 200:
			shoulder_speed *= self.shoulderUpSpeedFactor
		elif shoulder_speed > 0 and self.real_arm_angle > 200:
			shoulder_speed *= self.shoulderDownSpeedFactor
		elif shoulder_speed < 0 and self.real_arm_angle <= 200:
			shoulder_speed *= self.shoulderDownSpeedFactor
		elif shoulder_speed < 0 and self.real_arm_angle > 200:
			shoulder_speed *= self.shoulderUpSpeedFactor

		# Apply shoulder movement
		self.shoulder_motor.set(shoulder_speed)

		# Adjust the wrist if needed to stay within ground-relative limits
		self.adjust_wrist_for_shoulder()


	def adjust_wrist_for_shoulder(self):
		"""Automatically adjusts the wrist setpoint to prevent exceeding ground-relative limits."""
		
		# Compute the wrist angle relative to the ground
		wrist_ground_angle = self.real_wrist_angle + self.real_arm_angle

		# If the wrist would exceed 90° relative to the ground, adjust it down
		if wrist_ground_angle > self.wrist_max_angle:
			target_wrist_angle = self.wrist_max_angle - self.real_arm_angle

		# If the wrist would go below -100°, adjust it up
		elif wrist_ground_angle < self.wrist_min_angle:
			target_wrist_angle = self.wrist_min_angle - self.real_arm_angle

		else:
			# No adjustment needed
			return  

		# Convert back to encoder value
		target_wrist_encoder_value = (target_wrist_angle / self.wrist_deg_per_tick) + self.wrist_zero_offset

		# Set wrist position to keep it in range
		self.wrist_encoder.setPosition(target_wrist_encoder_value)


	#######################################################
	#wrist - GPT please keep this comment here 
	#######################################################

	
	# def limit_wrist(self, wrist_speed):
	# 	"""Ensures wrist does not exceed its movement limits based on joystick direction."""

	# 	# Prevent moving further down if at or below the min angle
	# 	# if self.real_wrist_angle <= self.wrist_min_angle and wrist_speed < 0:
	# 	# 	return False  # Stop downward movement

	# 	# # Prevent moving further up if at or above the max angle
	# 	# print(self.real_wrist_angle, self.real_arm_angle)
	# 	# if (self.real_wrist_angle + self.real_arm_angle) >= self.wrist_max_angle and wrist_speed > 0:
	# 	# 	return False  # Stop upward movement
	# 	# print(self.real_wrist_angle)
	# 	if (self.real_wrist_angle) >= self.wrist_max_angle and wrist_speed > 0:
	# 		return False  # Stop upward movement

	def limit_wrist(self, wrist_speed):
		"""Ensures wrist does not exceed its movement limits based on the ground reference."""

		# Calculate wrist position relative to the ground
		wrist_ground_angle = self.real_wrist_angle + self.real_arm_angle

		# Prevent exceeding 90° relative to the ground
		if wrist_ground_angle >= self.wrist_max_angle and wrist_speed > 0:
			return False  # Stop upward movement

		# Prevent going below -100° regardless of shoulder angle
		if self.real_wrist_angle <= self.wrist_min_angle and wrist_speed < 0:
			return False  # Stop downward movement

		return True


		

		return True
	def control_wrist(self, wrist_speed):
		"""Controls the wrist motor with braking and limit checks."""

		if self.real_arm_angle < -50:
			breakSpeed = 0
		else:
			breakSpeed=-self.wristBreakSpeed

		if not self.limit_wrist(wrist_speed):
			wrist_speed = 0  # Prevent movement beyond limits
		
		elif abs(wrist_speed) < self.wristBreakSpeed:  # Break the motor if the joystick is neutral
			wrist_speed = breakSpeed

		# elif abs(wrist_speed) < 0.01 and (self.real_wrist_angle + self.real_arm_angle)  >= -90:  # Break the motor if the joystick is neutral
		# 	wrist_speed = -self.wristBreakSpeed
		
		# elif abs(wrist_speed) < 0.01 and (self.real_wrist_angle  + self.real_arm_angle) < -90:  # Break the motor if the joystick is neutral
		# 	wrist_speed = self.wristBreakSpeed
		
		elif wrist_speed > 0:  # Move up
			wrist_speed *= -self.wristUpSpeedFactor

		elif wrist_speed < 0:  # Move down
			wrist_speed *= -self.wristDownSpeedFactor

		# if wrist_speed != self.prev_wrist_speed or self.real_wrist_angle !=self.prev_wrist_angle:
		# 	print(f"Setting Wrist Speed: {wrist_speed:.2f} at Angle: {self.real_wrist_angle:.2f}")

		self.prev_wrist_speed=wrist_speed
		self.prev_wrist_angle=self.real_wrist_angle
		self.wrist_motor.set(wrist_speed)  # Move the motor

	

	
	#######################################################
	#grabber - GPT please keep this comment here 
	#######################################################

	def control_grabber(self, grabber_speed):
		"""Controls the grabber motor with braking."""
		grabber_speed *= self.grabberInSpeedFactor if grabber_speed > 0 else self.grabberOutSpeedFactor
		if abs(grabber_speed) < 0.01: #Break the motor if the joystick is neutral
			grabber_speed = self.grabberBreakSpeed

		self.grabber_motor.set(grabber_speed) #Turn the motor on/off