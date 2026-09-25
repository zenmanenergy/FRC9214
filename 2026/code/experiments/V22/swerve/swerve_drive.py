"""Main swerve drive control system.

Coordinates all four swerve wheels, odometry, and IMU for field-relative and
robot-relative movement. Supports teleop drive, autonomous path following,
and in-place rotation with motor current monitoring for safety.

Path Recording for Debugging:
  The library can record the actual robot path during autonomous execution
  to compare against the planned path. This helps debug odometry accuracy
  and trajectory tuning.

Example - Teleop:
    swerve = SwerveDrive()
    # Field-relative drive
    swerve.drive_swerve(forward=0.5, strafe=0.2, rotate=0.1)
    # Update position estimate
    swerve.odometry.update()
    # Fuse IMU heading into odometry
    swerve.imu.fuse_heading(swerve.odometry)

Example - Autonomous with Recording:
    swerve = SwerveDrive()
    swerve.start_recording()  # Begin recording actual path
    swerve.follow_path(waypoints, speed=0.6)
    while not swerve.is_path_complete():
        swerve.update_autonomous()  # Automatically records position
    
    # Export for analysis
    actual = swerve.get_recorded_path()
    planned = list(swerve.path.sample_path(step_cm=5.0))
    swerve.export_recorded_path("/home/lvuser/path_run1.json")
"""

from typing import Dict, Optional, Tuple, List
import wpilib
from wpilib import SmartDashboard, RobotController
from .swerve_wheel import SwerveWheel
from .encoder_calibration import EncoderCalibration
from .pid_controller import PIDController
from .swerve_odometry import SwerveOdometry
from .swerve_imu import SwerveIMU
from .swerve_tune import SwerveTuner
from .catmull_rom import CatmullRomSpline
from . import swerve_config as config
import math


class SwerveDrive:
	
	def __init__(self) -> None:
		"""Initialize the swerve drive system, loading calibration and configuring motors."""
		
		self.wheels = {}
		for wheel_name, pin_config in config.WHEELS.items():
			self.wheels[wheel_name] = SwerveWheel(
				wheel_name,
				pin_config["drive_canid"],
				pin_config["turn_canid"],
				pin_config["encoder_dio"],
				pin_config["manual_offset"]
			)
		
		self.calibration = EncoderCalibration()
		for wheel_name, offset in self.calibration.offsets.items():
			if wheel_name in self.wheels:
				self.wheels[wheel_name].offset = offset
		
		self.alignment_gains = self.calibration.get_alignment_gains()
		
		self.aligning = False
		self.align_start_time = None
		self.target_align_angle = 0
		
		battery_voltage = RobotController.getBatteryVoltage()
		pid_gains = self.calibration.get_interpolated_gains(battery_voltage)
		
		self.pid_controllers = {}
		for wheel_name in self.wheels.keys():
			if isinstance(pid_gains, dict) and wheel_name in pid_gains:
				kp = pid_gains[wheel_name]["kp"]
				ki = pid_gains[wheel_name]["ki"]
				kd = pid_gains[wheel_name]["kd"]
			else:
				kp = pid_gains.get("kp", 0.003)
				ki = pid_gains.get("ki", 0.005)
				kd = pid_gains.get("kd", 0.0001)
			
			self.pid_controllers[wheel_name] = PIDController(
				kp=kp,
				ki=ki,
				kd=kd,
				name=f"Wheel_{wheel_name}"
			)
		
		self.odometry = SwerveOdometry(self.wheels)
		self.imu = SwerveIMU()
		self.tuner = SwerveTuner(self.wheels, self.pid_controllers, self.calibration)
		self.tuner.publish_tuning_history()
		
		self.wheel_alignment_state = {}
		
		self.previous_target_angles = {}
		
		self.movement_state = "idle"
		self.last_move_time = 0
		self.per_wheel_previous_power = {name: 0.0 for name in self.wheels.keys()}
		self.power_ramp_rate = 0.2
		
		self.motor_current_max = 45.0
		self.motor_current_sustained_limit = 35.0
		self.motor_current_history = {name: [] for name in self.wheels.keys()}
		self.motor_current_max_history = 10
		self.motor_current_alert_cooldown = 0
		self.motor_current_alerts = {name: False for name in self.wheels.keys()}
		
		# Autonomous path following state
		self.path = None
		self.path_current_distance = 0.0
		self.path_speed = 0.5
		self.following_path = False
		
		# Path recording for accuracy debugging
		self.recording_path = False
		self.recorded_positions = []
	
	def stop_all(self) -> None:
		for wheel in self.wheels.values():
			wheel.stop()
		self.wheel_alignment_state.clear()
		self.movement_state = "idle"
		self.per_wheel_previous_power = {name: 0.0 for name in self.wheels.keys()}
	
	# ========== AUTONOMOUS PATH FOLLOWING ==========
	
	def follow_path(self, waypoints: List[Dict], speed: float = 0.5) -> None:
		"""Start following a path defined by waypoints.
		
		Waypoints should have 'x', 'y', and 'heading' keys in centimeters and degrees.
		Call update_autonomous() in the main loop to drive along the path.
		
		Args:
			waypoints: List of {'x': cm, 'y': cm, 'heading': degrees}
			speed: Drive speed 0.0 to 1.0 (default 0.5)
		
		Example:
			waypoints = [
				{'x': 0, 'y': 0, 'heading': 0},
				{'x': 100, 'y': 50, 'heading': 45},
				{'x': 200, 'y': 100, 'heading': 90},
			]
			swerve.follow_path(waypoints, speed=0.6)
			
			while not swerve.is_path_complete():
				swerve.update_autonomous()
				odometry.update()
				imu.fuse_heading(odometry)
		"""
		self.path = CatmullRomSpline(waypoints)
		self.path_current_distance = 0.0
		self.path_speed = speed
		self.following_path = True
		self.movement_state = "moving"
		print(f"[AUTONOMOUS] Following path with {len(waypoints)} waypoints, "
			  f"total distance {self.path.get_total_distance():.1f}cm", flush=True)
	
	def update_autonomous(self) -> None:
		"""Update robot position along the current path.
		
		Call this once per loop while following a path.
		Automatically drives toward waypoints and updates heading.
		
		Example:
			while not swerve.is_path_complete():
				swerve.update_autonomous()
		"""
		if not self.following_path or self.path is None:
			self.stop_all()
			return
		
		# Get target position at current distance
		state = self.path.get_state_at_distance(self.path_current_distance)
		target_x = state['x']
		target_y = state['y']
		target_heading = state['heading']
		
		# Get current position
		current_x, current_y = self.odometry.get_position()
		current_heading = self.odometry.get_heading()
		
		# Calculate vector to target
		dx = target_x - current_x
		dy = target_y - current_y
		distance_to_target = math.sqrt(dx * dx + dy * dy)
		
		# If we're close enough to waypoint, advance distance
		if distance_to_target < 10.0:  # Within 10cm
			self.path_current_distance += 10.0
			# Check if path is complete
			if self.path_current_distance >= self.path.get_total_distance():
				self.following_path = False
				self.stop_all()
				print("[AUTONOMOUS] Path complete!", flush=True)
				return
			# Re-query with new distance
			state = self.path.get_state_at_distance(self.path_current_distance)
			target_x = state['x']
			target_y = state['y']
			target_heading = state['heading']
			dx = target_x - current_x
			dy = target_y - current_y
		
		# Calculate heading toward target
		target_movement_angle = math.degrees(math.atan2(dy, dx))
		if target_movement_angle < 0:
			target_movement_angle += 360
		
		# Drive toward target and maintain path heading
		# Use drive_swerve with computed forward/strafe
		forward = min(self.path_speed, self.path_speed * (distance_to_target / 50.0))
		
		# Convert field-frame target to robot-relative
		angle_diff = target_movement_angle - current_heading
		if angle_diff > 180:
			angle_diff -= 360
		elif angle_diff < -180:
			angle_diff += 360
		
		strafe = 0.0  # Keep forward motion
		rotate = 0.0  # Use heading from path
		
		# Drive and maintain target heading
		self.drive_swerve(forward, strafe, rotate)
		self.odometry.update()
		self.imu.fuse_heading(self.odometry)
		self.record_position()  # Record actual position if recording enabled
		self.update_motor_currents()
	
	def is_path_complete(self) -> bool:
		"""Check if path following is complete.
		
		Returns:
			True if path is done or no path is being followed
		
		Example:
			if swerve.is_path_complete():
				print("Reached destination!")
		"""
		return not self.following_path
	
	def stop_path(self) -> None:
		"""Cancel path following immediately."""
		self.following_path = False
		self.path = None
		self.stop_all()
		print("[AUTONOMOUS] Path cancelled", flush=True)
	
	# ========== PATH RECORDING FOR DEBUGGING ==========
	
	def start_recording(self) -> None:
		"""Start recording the actual robot path during autonomous.
		
		Call before follow_path() to capture how the robot actually moves.
		
		Example:
			swerve.start_recording()
			swerve.follow_path(waypoints, speed=0.6)
			while not swerve.is_path_complete():
				swerve.update_autonomous()
			
			actual_path = swerve.get_recorded_path()
		"""
		self.recorded_positions = []
		self.recording_path = True
		print("[RECORDING] Started path recording", flush=True)
	
	def stop_recording(self) -> None:
		"""Stop recording the robot path."""
		self.recording_path = False
		print(f"[RECORDING] Stopped path recording ({len(self.recorded_positions)} points)", flush=True)
	
	def record_position(self) -> None:
		"""Record current robot position. Called automatically during update_autonomous().
		
		Can also be called manually during any movement for general path tracking.
		"""
		if not self.recording_path:
			return
		
		x, y = self.odometry.get_position()
		heading = self.odometry.get_heading()
		timestamp = wpilib.Timer.getFPGATimestamp()
		
		self.recorded_positions.append({
			'x': x,
			'y': y,
			'heading': heading,
			'timestamp': timestamp,
			'distance': self.odometry.get_total_distance(),
		})
	
	def get_recorded_path(self) -> List[Dict]:
		"""Get the recorded path as a list of position dictionaries.
		
		Returns:
			List of dicts with keys: 'x', 'y', 'heading', 'timestamp', 'distance'
		
		Example:
			actual = swerve.get_recorded_path()
			planned = swerve.path.sample_path(step_cm=5.0)  # or from follow_path
		"""
		return self.recorded_positions.copy()
	
	def clear_recording(self) -> None:
		"""Clear recorded path data."""
		self.recorded_positions = []
		self.recording_path = False
	
	def export_recorded_path(self, filename: str = "/tmp/recorded_path.json") -> bool:
		"""Export recorded path to JSON file.
		
		Args:
			filename: File path to write JSON to (default: /tmp/recorded_path.json)
		
		Returns:
			True if export successful, False if recording is empty or write failed
		
		Example:
			if swerve.export_recorded_path("/home/lvuser/paths/run1.json"):
				print("Path exported successfully")
		"""
		if not self.recorded_positions:
			print("[RECORDING] No recorded path to export", flush=True)
			return False
		
		try:
			import json
			data = {
				'waypoint_count': len(self.recorded_positions),
				'start_position': self.recorded_positions[0] if self.recorded_positions else None,
				'end_position': self.recorded_positions[-1] if self.recorded_positions else None,
				'positions': self.recorded_positions,
			}
			
			with open(filename, 'w') as f:
				json.dump(data, f, indent=2)
			
			print(f"[RECORDING] Exported {len(self.recorded_positions)} positions to {filename}", flush=True)
			return True
		except Exception as e:
			print(f"[RECORDING] Failed to export path: {e}", flush=True)
			return False
	
	def publish_path_to_dashboard(self) -> None:
		"""Publish recorded and planned paths to SmartDashboard for visualization.
		
		This sends both the actual path taken and the planned path to SmartDashboard,
		allowing your dashboard to display them together for comparison.
		
		Call this after autonomous completes to see comparison data:
		
		Example:
			swerve.start_recording()
			swerve.follow_path(waypoints, speed=0.6)
			
			while not swerve.is_path_complete():
				swerve.update_autonomous()
			
			swerve.publish_path_to_dashboard()  # Send to dashboard
		
		Publishes to SmartDashboard:
			- path/recorded/count: Number of recorded positions
			- path/recorded/x: Array of X coordinates
			- path/recorded/y: Array of Y coordinates
			- path/recorded/heading: Array of headings
			- path/planned/count: Number of planned waypoints
			- path/planned/x: Array of planned X coordinates
			- path/planned/y: Array of planned Y coordinates
			- path/planned/heading: Array of planned headings
		"""
		try:
			import json
			
			# Publish recorded path
			if self.recorded_positions:
				recorded_x = [p['x'] for p in self.recorded_positions]
				recorded_y = [p['y'] for p in self.recorded_positions]
				recorded_heading = [p['heading'] for p in self.recorded_positions]
				
				SmartDashboard.putNumber("path/recorded/count", len(self.recorded_positions))
				SmartDashboard.putString("path/recorded/x", json.dumps(recorded_x))
				SmartDashboard.putString("path/recorded/y", json.dumps(recorded_y))
				SmartDashboard.putString("path/recorded/heading", json.dumps(recorded_heading))
				print(f"[DASHBOARD] Published {len(self.recorded_positions)} recorded positions", flush=True)
			
			# Publish planned path
			if self.path:
				planned_positions = list(self.path.sample_path(step_cm=5.0))
				planned_x = [p['x'] for p in planned_positions]
				planned_y = [p['y'] for p in planned_positions]
				planned_heading = [p['heading'] for p in planned_positions]
				
				SmartDashboard.putNumber("path/planned/count", len(planned_positions))
				SmartDashboard.putString("path/planned/x", json.dumps(planned_x))
				SmartDashboard.putString("path/planned/y", json.dumps(planned_y))
				SmartDashboard.putString("path/planned/heading", json.dumps(planned_heading))
				print(f"[DASHBOARD] Published {len(planned_positions)} planned positions", flush=True)
			
		except Exception as e:
			print(f"[DASHBOARD] Failed to publish paths: {e}", flush=True)
	
	# ========== END AUTONOMOUS ==========
	
	def _apply_smooth_acceleration(self, wheel_name: str, target_power: float) -> float:
		previous_power = self.per_wheel_previous_power.get(wheel_name, 0.0)
		
		# Only ramp when decelerating to zero (stopping smoothly).
		# Acceleration is handled by the caller (navigator ramp or joystick).
		# Applying a ramp on the way up fights the navigator's own velocity profile.
		if target_power == 0.0:
			power_diff = target_power - previous_power
			ramped_power = previous_power + (power_diff * self.power_ramp_rate)
		else:
			ramped_power = target_power
		
		self.per_wheel_previous_power[wheel_name] = ramped_power
		
		return ramped_power
	
	def is_moving(self) -> bool:
		"""True if robot is moving or aligning wheels."""
		return self.movement_state in ["moving", "aligning"]
	
	def is_aligning(self) -> bool:
		"""True if wheels are currently aligning to target angles."""
		return len(self.wheel_alignment_state) > 0
	
	def get_movement_state(self) -> str:
		"""Return current movement state: 'idle', 'moving', or 'aligning'."""
		return self.movement_state
	
	def update_motor_currents(self):
		current_time = wpilib.Timer.getFPGATimestamp()
		
		for wheel_name, wheel in self.wheels.items():
			try:
				drive_current = wheel.drive_motor.getOutputCurrent() if wheel.drive_motor else 0.0
				turn_current = wheel.turn_motor.getOutputCurrent() if wheel.turn_motor else 0.0
				
				motor_current = max(drive_current, turn_current)
				
				self.motor_current_history[wheel_name].append({
					"current": motor_current,
					"time": current_time
				})
				
				if len(self.motor_current_history[wheel_name]) > self.motor_current_max_history:
					self.motor_current_history[wheel_name].pop(0)
				
				if motor_current > self.motor_current_max:
					if not self.motor_current_alerts[wheel_name]:
						#print(f"[CURRENT-ALERT] {wheel_name}: SPIKE {motor_current:.1f}A (collision/bind?)", flush=True)
						self.motor_current_alerts[wheel_name] = True
						self.motor_current_alert_cooldown = 10
				elif motor_current > self.motor_current_sustained_limit:
					avg_current = sum([r["current"] for r in self.motor_current_history[wheel_name]]) / len(self.motor_current_history[wheel_name])
					if avg_current > self.motor_current_sustained_limit:
						if not self.motor_current_alerts[wheel_name]:
							print(f"[CURRENT-ALERT] {wheel_name}: SUSTAINED HIGH {avg_current:.1f}A avg (mechanical issue?)", flush=True)
							self.motor_current_alerts[wheel_name] = True
							self.motor_current_alert_cooldown = 10
				else:
					if self.motor_current_alerts[wheel_name]:
						#print(f"[CURRENT-CLEAR] {wheel_name}: Current normalized", flush=True)
						self.motor_current_alerts[wheel_name] = False
				
				SmartDashboard.putNumber(f"{wheel_name}_motor_current", motor_current)
				SmartDashboard.putBoolean(f"{wheel_name}_current_alert", self.motor_current_alerts[wheel_name])
				
			except Exception:
				pass
		
		if self.motor_current_alert_cooldown > 0:
			self.motor_current_alert_cooldown -= 1
	
	def get_motor_current(self, wheel_name):
		if wheel_name in self.motor_current_history and self.motor_current_history[wheel_name]:
			return self.motor_current_history[wheel_name][-1]["current"]
		return 0.0
	
	def has_current_alert(self, wheel_name=None):
		if wheel_name:
			return self.motor_current_alerts.get(wheel_name, False)
		return any(self.motor_current_alerts.values())
	
	def drive_straight(self, speed: float, target_angle: float = 0.0) -> None:
		"""Drive in a straight line at a constant angle.
		
		Args:
			speed: Forward speed (-1.0 to 1.0)
			target_angle: Heading to maintain in degrees (0-360)
		"""
		if abs(speed) < 0.01:
			for wheel_name, wheel in self.wheels.items():
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
			return
		
		for wheel_name, wheel_data in config.WHEELS.items():
			wheel_pos = wheel_data["position"]
			target_wheel_angle = target_angle
			
			target_wheel_angle = (target_wheel_angle + wheel_data["manual_offset"]) % 360
			
			self.drive_wheel_to_angle(wheel_name, target_wheel_angle)
			
			wheel = self.wheels[wheel_name]
			current_angle = wheel.get_angle()
			angle_error = abs(target_wheel_angle - current_angle)
			if angle_error > 180:
				angle_error = 360 - angle_error
			
			tolerance = config.ALIGN_TOLERANCE
			if angle_error <= tolerance:
				drive_power = -speed * config.MOTOR_SCALE_TELEOP
				ramped_power = self._apply_smooth_acceleration(wheel_name, drive_power)
				wheel.set_drive_power(ramped_power)
				self.movement_state = "moving"
				self.last_move_time = wpilib.Timer.getFPGATimestamp()
			else:
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
				self.movement_state = "aligning"
		
		self.update_single_wheel_alignment()
		self.update_motor_currents()
	
	def drive_to_heading(self, target_angle: float) -> bool:
		"""Rotate in place to face a target heading.
		
		Args:
			target_angle: Desired heading in degrees (0-360)
			
		Returns:
			True when all wheels are aligned and rotation complete
		"""
		angle_diff = target_angle - 0
		if angle_diff > 180:
			angle_diff -= 360
		elif angle_diff < -180:
			angle_diff += 360
		
		rotation_speed = 0.5
		drive_power = -abs(rotation_speed) * config.MOTOR_SCALE_TELEOP
		
		all_aligned = True
		for wheel_name in config.WHEELS.keys():
			base_angle = config.WHEELS[wheel_name]["rotation_angle"]
			if angle_diff >= 0:
				target_wheel_angle = base_angle
			else:
				target_wheel_angle = (base_angle + 180) % 360
			
			target_wheel_angle = (target_wheel_angle + config.WHEELS[wheel_name]["manual_offset"]) % 360
			
			self.drive_wheel_to_angle(wheel_name, target_wheel_angle)
			
			wheel = self.wheels[wheel_name]
			current_angle = wheel.get_angle()
			angle_error = abs(target_wheel_angle - current_angle)
			if angle_error > 180:
				angle_error = 360 - angle_error
			
			tolerance = config.ALIGN_TOLERANCE
			if angle_error <= tolerance:
				ramped_power = self._apply_smooth_acceleration(wheel_name, drive_power)
				wheel.set_drive_power(ramped_power)
			else:
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
				all_aligned = False
				self.movement_state = "aligning"
		
		self.update_single_wheel_alignment()
		self.update_motor_currents()
		
		return all_aligned
	
	def drive_for_distance(self, speed: float, target_distance_cm: float, target_angle: float = 0.0) -> bool:
		"""Drive a specific distance with automatic deceleration at endpoint.
		
		Args:
			speed: Speed to maintain (0.0 to 1.0)
			target_distance_cm: Distance to travel in centimeters
			target_angle: Heading to maintain during drive
			
		Returns:
			True when target distance reached
		"""
		current_distance = self.odometry.get_distance_traveled()
		remaining_distance = target_distance_cm - current_distance
		
		if remaining_distance <= 0:
			self.drive_straight(0.0, target_angle)
			return True
		
		decel_distance = 20.0
		if remaining_distance < decel_distance:
			ramped_speed = speed * (remaining_distance / decel_distance)
		else:
			ramped_speed = speed
		
		self.drive_straight(ramped_speed, target_angle)
		return False
	
	def drive_swerve(self, forward: float, strafe: float, rotate: float) -> None:
		"""Execute swerve drive with joystick inputs (field-relative).
		
		Args:
			forward: Forward/backward component (-1.0 to 1.0)
			strafe: Left/right component (-1.0 to 1.0)
			rotate: Rotation component (-1.0 to 1.0)
		"""
		# Only apply deadzone to rotate - joystick deadband already handled in PilotJoystick.
		# Applying deadzone to forward/strafe here breaks autonomous ramp-up (small values get zeroed).
		rotate = rotate if abs(rotate) >= 0.1 else 0.0
		
		if forward == 0 and strafe == 0 and rotate == 0:
			for wheel_name, wheel in self.wheels.items():
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
			if not self.is_aligning():
				self.movement_state = "idle"
			return
		
		forward *= -1
		strafe *= -1
		
		wheel_vectors = {}
		
		for wheel_name, wheel_data in config.WHEELS.items():
			wheel_pos = wheel_data["position"]
			vx = -forward - rotate * wheel_pos["x"]
			vy = strafe - rotate * wheel_pos["y"]
			
			angle = math.degrees(math.atan2(vy, vx))
			
			if angle < 0:
				angle += 360
			
			angle = (angle + 180) % 360
			angle = (angle + config.WHEELS[wheel_name]["manual_offset"]) % 360
			
			wheel_speed = math.sqrt(vx*vx + vy*vy)
			
			wheel_vectors[wheel_name] = {"speed": wheel_speed, "angle": angle}
		
		angle_hash = tuple(sorted([(name, round(data['angle'], 1)) for name, data in wheel_vectors.items()]))
		
		max_wheel_speed = max([data['speed'] for data in wheel_vectors.values()]) if wheel_vectors else 0
		if max_wheel_speed > 1.0:
			speed_scale = 1.0 / max_wheel_speed
			for wheel_data in wheel_vectors.values():
				wheel_data['speed'] *= speed_scale
		
		for wheel_name, wheel_data in wheel_vectors.items():
			wheel = self.wheels[wheel_name]
			target_angle = wheel_data["angle"]
			target_angle = (target_angle + 180) % 360
			target_speed = wheel_data["speed"]
			
			current_angle = wheel.get_angle()
			
			# Signed shortest-path error
			raw_error = target_angle - current_angle
			if raw_error > 180:
				raw_error -= 360
			elif raw_error < -180:
				raw_error += 360
			
			# If turning more than 90 degrees, flip the wheel 180 and reverse drive instead
			if abs(raw_error) > 90:
				target_angle = (target_angle + 180) % 360
				target_speed = -target_speed
				raw_error = raw_error - 180 if raw_error > 0 else raw_error + 180
			
			angle_error = abs(raw_error)
			
			# Always register the target so wheels re-align if they drift away
			self.drive_wheel_to_angle(wheel_name, target_angle)
			
			# Scale drive power by cosine of angle error.
			# cos(0) = 1.0 (full power when aligned), cos(90) = 0.0 (no power when perpendicular).
			# This replaces the hard on/off cutoff which caused pulsing during combined drive+rotate.
			angle_scale = math.cos(math.radians(angle_error))
			if angle_scale < 0:
				angle_scale = 0.0
			
			drive_power = -target_speed * config.MOTOR_SCALE_TELEOP * angle_scale
			ramped_power = self._apply_smooth_acceleration(wheel_name, drive_power)
			wheel.set_drive_power(ramped_power)
			if abs(drive_power) > 0.01:
				self.movement_state = "moving"
				self.last_move_time = wpilib.Timer.getFPGATimestamp()
			elif len(self.wheel_alignment_state) > 0:
				self.movement_state = "aligning"
		
		# Debug: log all wheel drive powers being sent
		drive_powers_str = " | ".join([f"{name}={self.wheels[name].get_drive_power():.3f}" for name in self.wheels.keys()])
		print(f"[DRIVE-SWERVE] Powers: {drive_powers_str}")
		
		self.update_motor_currents()
	
	def rotate_in_place_autotune(self, rotation_power):
		"""
		Rotation autotune wrapper - delegates to drive_rotation() which correctly:
		1. Targets rotation stance angles from config
		2. Calls update_single_wheel_alignment() to maintain wheel angles via turn motors
		3. Applies relay power to DRIVE motors to physically rotate the robot
		
		Requires robot to be on the ground so heading actually changes.
		"""
		self.drive_rotation(rotation_power)
	
	def rotate_to_angle(self, angle):
		if not self.aligning:
			self.start_alignment(angle)
	
	def set_wheel_drive_power(self, wheel_name, power):
		if wheel_name in self.wheels:
			self.wheels[wheel_name].set_drive_power(power)
	
	def set_wheel_turn_power(self, wheel_name, power):
		if wheel_name in self.wheels:
			self.wheels[wheel_name].set_turn_power(power)
	
	def get_wheel_angle(self, wheel_name):
		if wheel_name in self.wheels:
			return self.wheels[wheel_name].get_angle()
		return -1
	
	def get_wheel_power(self, wheel_name):
		if wheel_name in self.wheels:
			return self.wheels[wheel_name].get_drive_power()
		return 0.0
	
	def set_wheel_zero(self, wheel_name):
		if wheel_name in self.wheels:
			wheel = self.wheels[wheel_name]
			wheel.set_zero_offset(wheel.get_raw_angle())
			self.calibration.set_offset(wheel_name, wheel.offset)
			self.calibration.save_offsets()
	
	def set_wheel_angle(self, wheel_name, target_angle):
		if wheel_name in self.wheels:
			wheel = self.wheels[wheel_name]
			raw = wheel.get_raw_angle()
			calculated_offset = (raw - target_angle) % 360
			
			wheel.offset = calculated_offset
			self.calibration.set_offset(wheel_name, calculated_offset)
			self.calibration.save_offsets()
			
			for name, offset in self.calibration.offsets.items():
				self.wheels[name].offset = offset
	
	def drive_wheel_to_angle(self, wheel_name, target_angle):
		if wheel_name not in self.wheels:
			return
		
		if wheel_name not in self.wheel_alignment_state or self.wheel_alignment_state[wheel_name]["target_angle"] != target_angle:
			self.wheel_alignment_state[wheel_name] = {
				"target_angle": target_angle,
				"start_time": wpilib.Timer.getFPGATimestamp()
			}
	
	
	def start_alignment(self, target_angle=0):
		self.wheel_alignment_state.clear()
		
		self.aligning = True
		self.align_start_time = wpilib.Timer.getFPGATimestamp()
		self.target_align_angle = target_angle
	
	def update_alignment(self):
		if not self.aligning:
			return
		
		elapsed = wpilib.Timer.getFPGATimestamp() - self.align_start_time
		
		if elapsed < config.ALIGN_TIMEOUT:
			all_aligned = True
			current_time = wpilib.Timer.getFPGATimestamp()
			
			for wheel_name, wheel in self.wheels.items():
				current_angle = wheel.get_angle()
				
				norm_current = current_angle
				if norm_current > 180:
					norm_current -= 360
				
				norm_target = self.target_align_angle
				if norm_target > 180:
					norm_target -= 360
				
				error = norm_target - norm_current
				if error > 180:
					error -= 360
				elif error < -180:
					error += 360
				
				pid = self.pid_controllers[wheel_name]
				pid_output = pid.calculate(error, current_time)
				
				speed = max(-config.MOTOR_SCALE_ALIGN, 
						   min(config.MOTOR_SCALE_ALIGN, pid_output))
				
				wheel.set_turn_power(speed)
				
				if abs(error) > config.ALIGN_TOLERANCE:
					all_aligned = False
			

			if all_aligned:
				self.aligning = False
				for pid in self.pid_controllers.values():
					pid.reset()
		else:
			self.stop_all()
			self.aligning = False
			for pid in self.pid_controllers.values():
				pid.reset()
	
	def update_single_wheel_alignment(self):
		if not self.wheel_alignment_state:
			return
		
		wheels_to_remove = []
		
		for wheel_name, align_info in self.wheel_alignment_state.items():
			target_angle = align_info["target_angle"]
			elapsed = wpilib.Timer.getFPGATimestamp() - align_info["start_time"]
			
			if wheel_name not in self.wheels:
				wheels_to_remove.append(wheel_name)
				continue
			
			wheel = self.wheels[wheel_name]
			raw_angle = wheel.get_raw_angle()
			current_angle = wheel.get_angle()
			
			raw_error = target_angle - current_angle
			
			if raw_error > 180:
				error = raw_error - 360
			elif raw_error < -180:
				error = raw_error + 360
			else:
				error = raw_error
			
			pid = self.pid_controllers[wheel_name]
			
			tolerance = config.ALIGN_TOLERANCE
			at_target = abs(error) < tolerance
			
			if at_target:
				wheel.turn_motor.set(0.0)
				pid.reset()
				wheels_to_remove.append(wheel_name)
				continue
			
			if elapsed > 30:
				wheel.turn_motor.set(0.0)
				wheels_to_remove.append(wheel_name)
				continue
			
			pid = self.pid_controllers[wheel_name]
			current_time = wpilib.Timer.getFPGATimestamp()
			pid_output = pid.calculate(error, current_time)
			
			speed = max(-config.MOTOR_SCALE_ALIGN, min(config.MOTOR_SCALE_ALIGN, pid_output))
			
			wheel.set_turn_power(speed)
		
		for wheel_name in wheels_to_remove:
			del self.wheel_alignment_state[wheel_name]
	
	def drive_rotation(self, rotation_input):
		if abs(rotation_input) < 0.1:
			for wheel_name, wheel in self.wheels.items():
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
			if not self.is_aligning():
				self.movement_state = "idle"
			return
		
		# Drive power direction follows rotation_input sign (positive = left, negative = right)
		drive_power = -rotation_input * config.MOTOR_SCALE_TELEOP
		
		for wheel_name in config.WHEELS.keys():
			# Always use base rotation angle, don't flip it based on direction
			# Just reverse motor direction like the forward stick does
			target_angle = (config.WHEELS[wheel_name]["rotation_angle"] + config.WHEELS[wheel_name]["manual_offset"]) % 360
			
			self.drive_wheel_to_angle(wheel_name, target_angle)
			
			wheel = self.wheels[wheel_name]
			current_angle = wheel.get_angle()
			angle_error = abs(target_angle - current_angle)
			if angle_error > 180:
				angle_error = 360 - angle_error
			
			tolerance = config.ALIGN_TOLERANCE
			if angle_error <= tolerance:
				ramped_power = self._apply_smooth_acceleration(wheel_name, drive_power)
				wheel.set_drive_power(ramped_power)
				if abs(rotation_input) > 0.01:
					self.movement_state = "moving"
					self.last_move_time = wpilib.Timer.getFPGATimestamp()
			else:
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
				if len(self.wheel_alignment_state) > 0:
					self.movement_state = "aligning"
		
		self.update_motor_currents()
		
		self.update_single_wheel_alignment()
		self.tuner.update()
	
	def rotate_in_place(self, rotation_power):
		"""Rotate robot in place at a given power (-1.0 to 1.0)"""
		self.drive_rotation(rotation_power)
	
	def start_autotune(self):
		self.tuner.start()
	
