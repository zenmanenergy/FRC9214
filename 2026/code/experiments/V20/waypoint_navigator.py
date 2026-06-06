"""Waypoint Navigator - Stage-based autonomous navigation system"""
import math
import time
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from swerve.pid_controller import PIDController
try:
	from wpilib import SmartDashboard
	_HAS_SD = True
except ImportError:
	_HAS_SD = False


class WaypointNavigator:
	"""
	Navigate robot through waypoints using a stage-based system:
	- Stage 0: Initialize
	- Stage 1: Rotate to target
	- Stage 2: Move forward
	- Stage 3: Complete (move to next waypoint)
	"""
	
	def __init__(self, swerve_drive):
		"""
		Initialize navigator.
		
		Args:
			swerve_drive: SwerveDrive instance
		"""
		self.drive = swerve_drive
		self.waypoints = []
		self.current_waypoint_index = 0
		self.stage = 0  # 0=idle, 1=rotate, 2=move, 3=done
		self.is_active = False
		self.start_time = None
		
		# Tuning parameters
		self.rotation_tolerance = 5.0  # degrees
		self.position_tolerance = 25.0  # cm - how close is "close enough"
		self.max_rotation_speed = 0.8  # power 0-1
		self.max_move_speed = 0.8  # power 0-1
		self.min_drive_speed = 0.15   # minimum above drive_swerve deadzone (0.1)
		self.timeout_per_stage = 10.0  # seconds

		# Velocity profiling (smooth accel/decel)
		self.current_drive_speed = 0.0   # tracked speed, ramped each loop
		self.accel_rate = 0.03           # max speed increase per loop tick (~50Hz = 1.5/sec)
		self.decel_rate = 0.05           # max speed decrease per loop tick (brakes harder)
		self.decel_distance = 80.0       # cm from final waypoint to start braking

		self.loop = False  # Loop through waypoints when all are complete

		# PID controllers
		# kD=0 intentionally: odometry dead-reckoning is noisy, D amplifies that noise
		# Tune kP_drive so that at your typical approach distance it outputs ~0.3-0.5
		# e.g. kp=0.004 -> at 100cm output=0.40, at 40cm output=0.16 (clamped to min)
		self.pid_drive = PIDController(kp=0.004, ki=0.0, kd=0.0, name="Nav_Drive")
		self.pid_drive.max_integral = 0.3
		# kp=0.007 -> at 45deg output=0.315, at 10deg output=0.07 (clamped to min)
		self.pid_rotate = PIDController(kp=0.007, ki=0.0, kd=0.0, name="Nav_Rotate")
		self.pid_rotate.max_integral = 0.3
		
		# Publish starting gains to SmartDashboard for live tuning
		self._publish_gains()
	
	def _publish_gains(self):
		"""Publish PID gains to SmartDashboard for live tuning"""
		if not _HAS_SD:
			return
		SmartDashboard.putNumber("Nav_kP_drive", self.pid_drive.kp)
		SmartDashboard.putNumber("Nav_kP_rotate", self.pid_rotate.kp)
		SmartDashboard.putNumber("Nav_pos_tolerance", self.position_tolerance)
		SmartDashboard.putNumber("Nav_rot_tolerance", self.rotation_tolerance)

	def _sync_gains(self):
		"""Read live gain edits from SmartDashboard and apply them"""
		if not _HAS_SD:
			return
		self.pid_drive.kp = SmartDashboard.getNumber("Nav_kP_drive", self.pid_drive.kp)
		self.pid_rotate.kp = SmartDashboard.getNumber("Nav_kP_rotate", self.pid_rotate.kp)
		self.position_tolerance = SmartDashboard.getNumber("Nav_pos_tolerance", self.position_tolerance)
		self.rotation_tolerance = SmartDashboard.getNumber("Nav_rot_tolerance", self.rotation_tolerance)

	def set_waypoints(self, waypoints):
		"""
		Set list of waypoints.
		
		Args:
			waypoints: List of dicts with 'x', 'y' keys (in cm)
		"""
		self.waypoints = waypoints
		self.current_waypoint_index = 0
		self.stage = 0
		print(f"[NAVIGATOR] Set {len(waypoints)} waypoints")
	
	def start(self):
		"""Start navigation sequence"""
		if not self.waypoints:
			print("[NAVIGATOR] No waypoints set!")
			return False
		
		self.is_active = True
		self.current_waypoint_index = 0
		self.stage = 1
		self.start_time = time.time()
		self.current_drive_speed = 0.0
		self.pid_drive.reset()
		self.pid_rotate.reset()
		print(f"[NAVIGATOR] Starting: {len(self.waypoints)} waypoints")
		return True
	
	def stop(self):
		"""Stop navigation and stop robot"""
		self.is_active = False
		self.stage = 0
		self.drive.stop_all()
		print("[NAVIGATOR] Stopped")
	
	def is_finished(self):
		"""Check if all waypoints completed"""
		return (self.current_waypoint_index >= len(self.waypoints)) and not self.is_active
	
	def get_status(self):
		"""Get current navigation status"""
		if not self.is_active:
			return {
				"active": False,
				"stage": 0,
				"waypoint": 0,
				"total": len(self.waypoints),
				"progress": "Idle"
			}
		
		current_wp = self.waypoints[self.current_waypoint_index]
		return {
			"active": True,
			"stage": self.stage,
			"waypoint": self.current_waypoint_index + 1,
			"total": len(self.waypoints),
			"target": current_wp,
			"current_x": self.drive.odometry.get_x(),
			"current_y": self.drive.odometry.get_y(),
			"heading": self.drive.odometry.get_heading(),
			"progress": f"Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} {'Driving' if self.stage == 1 else 'Dwell'}"
		}
	
	def update(self):
		"""Call periodically (e.g., from robot loop) to execute navigation"""
		if not self.is_active:
			return
		
		# Pick up any live gain changes from SmartDashboard
		self._sync_gains()
		
		# Get current pose
		current_x = self.drive.odometry.get_x()
		current_y = self.drive.odometry.get_y()
		current_heading = self.drive.odometry.get_heading()
		
		# Get target waypoint
		target = self.waypoints[self.current_waypoint_index]
		target_x = target["x"]
		target_y = target["y"]
		target_heading = target.get("heading", None)  # Use specified heading if provided
		
		# Calculate distance and angle to target
		dx = target_x - current_x
		dy = target_y - current_y
		distance_to_target = math.sqrt(dx**2 + dy**2)
		
		# Use specified heading if provided, otherwise calculate angle to target
		if target_heading is not None:
			target_angle = target_heading
		else:
			target_angle = math.degrees(math.atan2(dy, dx))
		
		# Normalize angles to 0-360
		target_angle = target_angle % 360
		current_heading = current_heading % 360
		
		# Calculate shortest rotation
		angle_diff = target_angle - current_heading
		if angle_diff > 180:
			angle_diff -= 360
		elif angle_diff < -180:
			angle_diff += 360
		
		elapsed = time.time() - self.start_time
		
		# ===== STAGE 1: DRIVE + ROTATE SIMULTANEOUSLY =====
		if self.stage == 1:
			if distance_to_target < self.position_tolerance:
				self.drive.stop_all()
				self.stage = 2
				self.start_time = time.time()
				print(f"[NAVIGATOR] WP {self.current_waypoint_index + 1}: Reached target (head_err={angle_diff:.1f}°)")
			elif elapsed > self.timeout_per_stage:
				self.drive.stop_all()
				print(f"[NAVIGATOR] WP {self.current_waypoint_index + 1}: Timeout (dist={distance_to_target:.1f}cm pos=({current_x:.1f},{current_y:.1f}) head_err={angle_diff:.1f}°)")
				self._advance_waypoint()
			else:
				# Find next mandatory stop: waypoint with dwell>0, or final waypoint (non-loop)
				next_stop_wp = None
				for look in range(self.current_waypoint_index, len(self.waypoints)):
					wp = self.waypoints[look]
					is_last = (look == len(self.waypoints) - 1) and not self.loop
					if wp.get("dwell", 0) > 0 or is_last:
						next_stop_wp = wp
						break
				
				if next_stop_wp is not None:
					dist_to_stop = math.sqrt((next_stop_wp["x"] - current_x)**2 + (next_stop_wp["y"] - current_y)**2)
					if dist_to_stop < self.decel_distance:
						# Decel to zero (full stop at dwell/end)
						target_speed = self.max_move_speed * (dist_to_stop / self.decel_distance)
						target_speed = max(0.0, target_speed)
					else:
						target_speed = self.max_move_speed
				else:
					target_speed = self.max_move_speed
				
				# Enforce min speed only when NOT in a decel-to-stop zone
				if target_speed > 0 and target_speed < self.min_drive_speed:
					target_speed = self.min_drive_speed
				
				# Ramp current_drive_speed toward target (accel limit up, decel limit down)
				if target_speed > self.current_drive_speed:
					self.current_drive_speed = min(target_speed, self.current_drive_speed + self.accel_rate)
				else:
					self.current_drive_speed = max(target_speed, self.current_drive_speed - self.decel_rate)
				
				drive_speed = self.current_drive_speed
				
				# Field-oriented translation toward target position
				h = math.radians(current_heading)
				norm_dx = dx / distance_to_target
				norm_dy = dy / distance_to_target
				robot_forward = norm_dx * math.sin(h) + norm_dy * math.cos(h)
				robot_strafe = norm_dx * math.cos(h) - norm_dy * math.sin(h)
				
				# Rotation PID: heading error -> rotation power
				if abs(angle_diff) < self.rotation_tolerance:
					rotation_power = 0.0
					self.pid_rotate.reset()
				else:
					rotation_power = self.pid_rotate.calculate(angle_diff)
					rotation_power = max(-self.max_rotation_speed, min(self.max_rotation_speed, rotation_power))
					# Enforce minimum power above drive_rotation deadzone (0.1)
					if 0 < abs(rotation_power) < 0.15:
						rotation_power = 0.15 if rotation_power > 0 else -0.15
				
				print(f"[NAVIGATOR] WP {self.current_waypoint_index + 1} - Dist {distance_to_target:.1f}cm Head {current_heading:.1f} Err {angle_diff:.1f} Spd {drive_speed:.3f} RF {robot_forward:.3f} RS {robot_strafe:.3f} Rot {rotation_power:.3f} T {elapsed:.2f}s")
				self.drive.drive_swerve(robot_forward * drive_speed, robot_strafe * drive_speed, rotation_power)
		
		# ===== STAGE 2: DWELL / ADVANCE =====
		elif self.stage == 2:
			dwell_time = target.get("dwell", 0)
			if elapsed > dwell_time:
				self._advance_waypoint()
	
	def _advance_waypoint(self):
		"""Move to next waypoint, loop, or finish"""
		self.current_waypoint_index += 1
		
		if self.current_waypoint_index >= len(self.waypoints):
			if self.loop:
				# Loop back to first waypoint
				self.current_waypoint_index = 0
				self.stage = 1
				self.start_time = time.time()				self.current_drive_speed = 0.0				self.pid_drive.reset()
				self.pid_rotate.reset()
				next_wp = self.waypoints[0]
				print(f"[NAVIGATOR] Looping back to WP 1: ({next_wp['x']}, {next_wp['y']})")
			else:
				# All waypoints complete
				self.drive.stop_all()
				self.is_active = False
				self.stage = 0
				print(f"[NAVIGATOR] Navigation complete!")
		else:
			# Next waypoint
			self.stage = 1
			self.start_time = time.time()
			# If we just stopped at a dwell, restart from zero speed
			if self.waypoints[self.current_waypoint_index - 1].get("dwell", 0) > 0:
				self.current_drive_speed = 0.0
			self.pid_drive.reset()
			self.pid_rotate.reset()
			next_wp = self.waypoints[self.current_waypoint_index]
			print(f"[NAVIGATOR] Moving to waypoint {self.current_waypoint_index + 1}: ({next_wp['x']}, {next_wp['y']})")
