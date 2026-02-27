# Minimal DIO Encoder Test - Rear Left wheel only (ABSOLUTE POSITIONING - Duty Cycle)
import wpilib
import math
import json
import os
from wpilib import Joystick
from rev import SparkMax, SparkLowLevel

class Robot(wpilib.TimedRobot):
	# Manual offsets per wheel (added to zero calibration)
	MANUAL_OFFSETS = {
		"front_right": 225.0,
		"rear_right": 315.0,
		"rear_left": 45.0,
		"front_left": 135.0
	}
	
	def robotInit(self):
		print("[ROBOT] Initializing 4-wheel encoder test...")
		
		# Define all 4 turning motors and encoders
		self.wheels = {
			"front_right": {
				"motor": SparkMax(9, SparkLowLevel.MotorType.kBrushless),
				"encoder": wpilib.DutyCycleEncoder(0),
				"button": 4,  # Y
				"canid": 9
			},
			"rear_right": {
				"motor": SparkMax(3, SparkLowLevel.MotorType.kBrushless),
				"encoder": wpilib.DutyCycleEncoder(1),
				"button": 2,  # B
				"canid": 3
			},
			"rear_left": {
				"motor": SparkMax(5, SparkLowLevel.MotorType.kBrushless),
				"encoder": wpilib.DutyCycleEncoder(2),
				"button": 1,  # A
				"canid": 5
			},
			"front_left": {
				"motor": SparkMax(7, SparkLowLevel.MotorType.kBrushless),
				"encoder": wpilib.DutyCycleEncoder(3),
				"button": 3,  # X
				"canid": 7
			}
		}
		print("[ROBOT] Initialized 4 motors and encoders")
		
		# Joystick
		self.joystick = Joystick(0)
		print("[ROBOT] Joystick initialized")
		
		# Test state
		self.focused = None  # Which wheel is currently focused
		self.focused_wheel = None  # Reference to the focused wheel data
		self.last_printed_angle = -999
		self.offsets = self.load_offsets()
		
		# Alignment state
		self.aligning = False
		self.align_start_time = None
		self.align_timeout = 5.0  # 5 seconds max to align
		
		print("[ROBOT] Ready. Press A/B/X/Y to focus on wheels.\n")
	
	def load_offsets(self):
		"""Load zero offsets from file"""
		defaults = {"front_right": 0.0, "rear_right": 0.0, "rear_left": 0.0, "front_left": 0.0}
		try:
			if os.path.exists("/home/lvuser/encoder_offsets.json"):
				with open("/home/lvuser/encoder_offsets.json", "r") as f:
					data = json.load(f)
					# Merge with defaults to ensure all wheels are present
					defaults.update(data)
					print(f"[LOAD] Loaded offsets: {defaults}")
					return defaults
		except:
			pass
		print("[LOAD] No offset file found, starting with all zeros")
		return defaults
	
	def save_offsets(self):
		"""Save all zero offsets to file"""
		try:
			with open("/home/lvuser/encoder_offsets.json", "w") as f:
				json.dump(self.offsets, f)
			print(f"[SAVE] Saved offsets: {self.offsets}")
		except Exception as e:
			print(f"[SAVE] Failed to save offsets: {e}")
	
	def testInit(self):
		print("[TEST] === ENTERING TEST MODE ===")
		print("[TEST] A = Focus rear left | B = Focus rear right | X = Focus front left | Y = Focus front right")
		print("[TEST] Right thumb left/right = Rotate wheel")
		print("[TEST] RB = Set this position as zero offset")
		print("[TEST] START = Show alignment info for all wheels to 0\n")
	
	def testPeriodic(self):
		# START button - align all wheels to 0 degrees
		if self.joystick.getRawButtonPressed(8):  # START is button 8
			self.aligning = True
			self.align_start_time = wpilib.Timer.getFPGATimestamp()
			print("[ALIGN] Starting alignment to 0 degrees for all wheels...")
			for wheel_name in self.wheels.keys():
				print(f"  {wheel_name}: moving to 0")
		
		# Handle alignment
		if self.aligning:
			elapsed = wpilib.Timer.getFPGATimestamp() - self.align_start_time
			
			if elapsed < self.align_timeout:
				all_aligned = True
				for wheel_name, wheel_data in self.wheels.items():
					offset = self.offsets[wheel_name]
					current_raw = wheel_data["encoder"].get() * 360.0
					current_angle = (current_raw - offset) % 360.0
					
					# Normalize to -180 to 180
					if current_angle > 180:
						current_angle -= 360
					
					# Simple P controller
					error = -current_angle  # Error to reach 0
					if abs(error) > 2:  # 2 degree tolerance
						all_aligned = False
						speed = max(-0.3, min(0.3, error * 0.002))  # Scale error to speed
						wheel_data["motor"].set(speed)
					else:
						wheel_data["motor"].set(0.0)
				
				if all_aligned:
					print("[ALIGN] All wheels aligned to 0!")
					self.aligning = False
			else:
				# Timeout
				print("[ALIGN] Alignment timeout!")
				for wheel_data in self.wheels.values():
					wheel_data["motor"].set(0.0)
				self.aligning = False
		
		# Check which wheel button is pressed (one at a time)
		for wheel_name, wheel_data in self.wheels.items():
			if self.joystick.getRawButtonPressed(wheel_data["button"]):
				if self.focused == wheel_name:
					# Toggle off if same wheel
					self.focused_wheel = None
					self.focused = None
					for w in self.wheels.values():
						w["motor"].set(0.0)
					print(f"[ZEROING] Focused off. Motor stopped.\n")
				else:
					# Focus on new wheel
					for w in self.wheels.values():
						w["motor"].set(0.0)  # Stop all motors
					self.focused = wheel_name
					self.focused_wheel = wheel_data
					self.last_printed_angle = -999
					print(f"[ZEROING] Focused on {wheel_name}. Use right thumb left/right to rotate.")
		
		# While focused: right thumb left/right controls motor
		if self.focused and self.focused_wheel:
			right_x = self.joystick.getRawAxis(4)  # Right joystick X
			self.focused_wheel["motor"].set(right_x * 0.05)  # Scale to 5% max speed
			
			# Get and print angle only when it changes
			angle_normalized = self.focused_wheel["encoder"].get()
			angle_raw_deg = angle_normalized * 360.0
			# Offset already includes manual offset, just subtract it
			angle_deg = (angle_raw_deg - self.offsets[self.focused]) % 360.0
			angle_int = int(round(angle_deg)) % 360
			
			if angle_int != self.last_printed_angle:
				print(f"{angle_int}")
				self.last_printed_angle = angle_int
		
		# RB button - set zero offset for focused wheel
		if self.joystick.getRawButtonPressed(6):  # RB is button 6
			if self.focused and self.focused_wheel:
				angle_normalized = self.focused_wheel["encoder"].get()
				angle_raw = angle_normalized * 360.0
				# Add manual offset to the raw reading before saving
				self.offsets[self.focused] = angle_raw + self.MANUAL_OFFSETS[self.focused]
				wheel_name = self.focused
				self.save_offsets()
				self.focused_wheel["motor"].set(0.0)
				self.focused = None
				self.focused_wheel = None
				print(f"[ZEROING] Saved offset: {self.offsets[wheel_name]:.1f} (raw + manual)")
				print("[ZEROING] Focused off. Motor stopped.\n")
			else:
				print("[ZEROING] Must be focused first (press A/B/X/Y to focus)")
	
	def testExit(self):
		self.aligning = False
		for wheel_data in self.wheels.values():
			wheel_data["motor"].set(0.0)
		print("[TEST] === EXITING TEST MODE ===\n")

if __name__ == "__main__":
	wpilib.run(Robot)
