"""Waypoint Navigator - Stage-based autonomous navigation system"""
import math
import time
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from swerve.pid_controller import PIDController
from swerve.encoder_calibration import EncoderCalibration
from swerve.catmull_rom import CatmullRomSpline
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
		
		# Load calibration and rotation gains
		try:
			self.calibration = EncoderCalibration()
			nav_gains = self.calibration.get_navigator_rotation_gains()
			kp = nav_gains.get("kp", 0.004)
			ki = nav_gains.get("ki", 0.0)
			kd = nav_gains.get("kd", 0.0001)
		except:
			kp = 0.004
			ki = 0.0
			kd = 0.0001
			self.calibration = None
		
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
		self.decel_distance = 150.0      # cm from final waypoint to start braking (increased for overshoot prevention)

		self.loop = False  # Loop through waypoints when all are complete

		# Spline-following mode (smooth continuous path)
		self.use_spline = False
		self.spline = None
		self.distance_traveled_along_spline = 0.0
		self.spline_total_distance = 0.0
		self.last_robot_x = 0.0
		self.last_robot_y = 0.0
		self.spline_start_x = 0.0
		self.spline_start_y = 0.0
		
		# Smoothing for spline following
		self.smooth_robot_forward = 0.0
		self.smooth_robot_strafe = 0.0
		self.smooth_desired_heading = 0.0

		# PID controllers
		# kD=0 intentionally: odometry dead-reckoning is noisy, D amplifies that noise
		# Tune kP_drive so that at your typical approach distance it outputs ~0.3-0.5
		# e.g. kp=0.004 -> at 100cm output=0.40, at 40cm output=0.16 (clamped to min)
		self.pid_drive = PIDController(kp=0.004, ki=0.0, kd=0.0, name="Nav_Drive")
		self.pid_drive.max_integral = 0.3
		# Load rotation gains from calibration (autotuned or manual tuning)
		self.pid_rotate = PIDController(kp=kp, ki=ki, kd=kd, name="Nav_Rotate")
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

	def set_waypoints(self, waypoints, use_spline=False):
		"""
		Set list of waypoints.
		
		Args:
			waypoints: List of dicts with 'x', 'y', 'heading' keys (in cm and degrees)
			use_spline: If True, interpolate smooth curve through waypoints (Catmull-Rom)
		"""
		self.waypoints = waypoints
		self.use_spline = use_spline
		self.current_waypoint_index = 0
		self.stage = 0
		self.distance_traveled_along_spline = 0.0
		self.last_robot_x = 0.0
		self.last_robot_y = 0.0
		self.smooth_robot_forward = 0.0
		self.smooth_robot_strafe = 0.0
		self.smooth_desired_heading = 0.0
		
		if use_spline and len(waypoints) >= 2:
			try:
				self.spline = CatmullRomSpline(waypoints)
				self.spline_total_distance = self.spline.get_total_distance()
				print(f"[NAVIGATOR] Created spline through {len(waypoints)} waypoints, total distance={self.spline_total_distance:.1f}cm")
			except Exception as e:
				print(f"[NAVIGATOR] Failed to create spline: {e}")
				self.use_spline = False
				self.spline = None
		
		print(f"[NAVIGATOR] Set {len(waypoints)} waypoints (spline={use_spline})")
	
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
	
	def autotune_rotation(self, target_heading=45.0, max_power=0.3, duration_seconds=10.0):
		"""
		Autotune rotation (heading) PID controller.
		
		Robot will attempt to reach target_heading from current heading using relay control,
		measuring oscillation to calculate optimal PID gains.
		
		Args:
			target_heading: Target heading in degrees (default 45.0)
			max_power: Maximum rotation power for autotune (default 0.5)
			duration_seconds: Maximum autotune time (default 15s)
		
		Returns:
			dict with tuning results including new kp, ki, kd and success status
		"""
		print("\n[NAVIGATOR-AUTOTUNE] Preparing rotation autotune...")
		print(f"[NAVIGATOR-AUTOTUNE] Target heading: {target_heading} degrees, max power: {max_power}, duration: {duration_seconds}s")
		print("[NAVIGATOR-AUTOTUNE] NOTE: Robot must be on the GROUND (not elevated) for this test.")
		print("[NAVIGATOR-AUTOTUNE] The robot must physically rotate for heading to change.")
		
		import time
		
		initial_heading = self.drive.odometry.get_heading()
		print(f"[NAVIGATOR-AUTOTUNE] Starting from heading: {initial_heading:.1f} degrees")
		
		# Setup phase: align wheels to rotation stance via drive_rotation()
		# drive_rotation() calls update_single_wheel_alignment() which actually moves turn motors
		print("[NAVIGATOR-AUTOTUNE] Aligning wheels to rotation stance (1.5 seconds)...")
		setup_start = time.time()
		while time.time() - setup_start < 1.5:
			self.drive.drive_rotation(0.2)
			time.sleep(0.05)
		
		# Stop before relay test begins
		self.drive.stop_all()
		time.sleep(0.3)
		print("[NAVIGATOR-AUTOTUNE] Wheels aligned. Starting relay test...")
		
		def get_heading_error():
			"""Get current heading error relative to target - reads IMU directly
			since odometry.get_heading() is stale when the main loop is blocked."""
			current = self.drive.imu.get_heading()
			error = target_heading - current
			# Normalize to [-180, 180]
			while error > 180:
				error -= 360
			while error < -180:
				error += 360
			return error
		
		# Disable acceleration ramping for relay test - relay needs instant direction reversals.
		# The ramp smoothly transitions over many cycles which prevents true oscillation.
		saved_ramp_rate = self.drive.power_ramp_rate
		self.drive.power_ramp_rate = 1.0
		self.drive.per_wheel_previous_power = {name: 0.0 for name in self.drive.wheels.keys()}
		
		def apply_rotation(power):
			# Use drive_rotation() which:
			# 1. Targets correct rotation stance angles from config
			# 2. Calls update_single_wheel_alignment() to maintain wheel angles
			# 3. Applies relay power to DRIVE motors (not turn motors)
			self.drive.drive_rotation(power)
		
		# Run autotune - target_cycles=2 is the minimum for a valid period measurement
		# (needs 2 zero crossings to calculate half-period)
		result = self.pid_rotate.autotune(
			get_error_func=get_heading_error,
			set_output_func=apply_rotation,
			max_power=max_power,
			duration_seconds=duration_seconds,
			target_cycles=2
		)
		
		# Restore ramp rate
		self.drive.power_ramp_rate = saved_ramp_rate
		self.drive.per_wheel_previous_power = {name: 0.0 for name in self.drive.wheels.keys()}
		
		# Print results
		if result['success']:
			print(f"\n[NAVIGATOR-AUTOTUNE] [OK] SUCCESS")
			print(f"[NAVIGATOR-AUTOTUNE] New gains: kp={result['kp']:.6f}, ki={result['ki']:.6f}, kd={result['kd']:.6f}")
			print(f"[NAVIGATOR-AUTOTUNE] Measured period={result['period']:.3f}s, amplitude={result['amplitude']:.2f} degrees")
			
			# Save to calibration file
			if self.calibration:
				try:
					self.calibration.set_navigator_rotation_gains(result['kp'], result['ki'], result['kd'])
					self.calibration.save_calibration()
					print(f"[NAVIGATOR-AUTOTUNE] [OK] Saved to {self.calibration.get_calibration_path()}")
				except Exception as e:
					print(f"[NAVIGATOR-AUTOTUNE] [FAIL] Failed to save: {e}")
		else:
			print(f"\n[NAVIGATOR-AUTOTUNE] [FAIL] FAILED: {result['message']}")
		
		return result
	
	def _update_spline(self):
		"""Update while following a Catmull-Rom spline (smooth continuous path)"""
		# Get current pose
		current_x = self.drive.odometry.get_x()
		current_y = self.drive.odometry.get_y()
		current_heading = self.drive.odometry.get_heading()
		
		# Update cumulative distance traveled along spline
		# Measure how far robot moved since last update
		if self.last_robot_x == 0 and self.last_robot_y == 0:
			# First update - initialize position
			self.last_robot_x = current_x
			self.last_robot_y = current_y
			self.spline_start_x = current_x
			self.spline_start_y = current_y
		
		# Distance traveled in this update
		dx_moved = current_x - self.last_robot_x
		dy_moved = current_y - self.last_robot_y
		distance_moved = math.sqrt(dx_moved**2 + dy_moved**2)
		
		# Accumulate distance with low-pass filter (0.7 = 70% new, 30% inertia)
		self.distance_traveled_along_spline += distance_moved * 0.7
		
		# Clamp to valid range [0, total_distance]
		self.distance_traveled_along_spline = max(0, min(self.distance_traveled_along_spline, self.spline_total_distance))
		
		# Remember position for next update
		self.last_robot_x = current_x
		self.last_robot_y = current_y
		
		# Find position along spline that matches cumulative distance
		seg_idx, t, actual_distance = self.spline.find_segment_for_distance(self.distance_traveled_along_spline)
		
		# Evaluate spline position and tangent at this point
		pos = self.spline.evaluate(seg_idx, t)
		tangent = self.spline.evaluate_tangent(seg_idx, t)
		
		# Tangent magnitude (for normalized direction)
		tangent_mag = math.sqrt(tangent['dx']**2 + tangent['dy']**2)
		if tangent_mag < 0.01:
			tangent_mag = 0.01  # Avoid divide by zero
		
		# Unit tangent vector (direction along the curve)
		tangent_x = tangent['dx'] / tangent_mag
		tangent_y = tangent['dy'] / tangent_mag
		
		# Desired heading from WAYPOINT HEADINGS, not tangent
		# This way if both waypoints have heading=0, desired_heading=0 throughout the path
		desired_heading = self.spline.interpolate_heading(seg_idx, t)
		
		# Calculate lateral error: how far off the spline path are we?
		# Vector from spline point to robot position
		error_x = current_x - pos['x']
		error_y = current_y - pos['y']
		
		# Decompose error into tangential and lateral components
		# Tangential: projection onto tangent direction (should be ~0 if we're on distance track)
		tangential_error = error_x * tangent_x + error_y * tangent_y
		
		# Lateral: perpendicular distance from path
		# Use cross product magnitude: |error × tangent|
		lateral_error = error_x * (-tangent_y) + error_y * tangent_x
		
		# Clamp lateral error to prevent overshooting
		lateral_error = max(-50, min(50, lateral_error))  # Max 50cm correction
		
		# Convert tangent and lateral error to robot frame
		h = math.radians(current_heading)
		cos_h = math.cos(h)
		sin_h = math.sin(h)
		
		# Forward: always move along tangent direction (tangent is in field frame)
		raw_robot_forward = tangent_x * sin_h + tangent_y * cos_h
		
		# Strafe: AGGRESSIVE correction based on lateral error
		# When drifting 20cm off path, strafe should be ~1.0 (full sideways)
		# Proportional gain: 0.05 = 5cm error = 0.25 strafe correction
		perp_x = -tangent_y
		perp_y = tangent_x
		lateral_correction = (lateral_error * 0.05)  # Aggressive: 20cm error = 1.0 strafe
		lateral_correction = max(-1.0, min(1.0, lateral_correction))  # Clamp to [-1.0, 1.0]
		
		# TRY: Negate lateral error direction - if robot overshoots right, strafe left
		raw_robot_strafe = (perp_x * sin_h + perp_y * cos_h) * (-lateral_correction)
		
		# Faster smoothing on strafe - need rapid response to lateral drift
		self.smooth_robot_strafe = self.smooth_robot_strafe * 0.3 + raw_robot_strafe * 0.7
		robot_forward = raw_robot_forward  # Forward follows tangent directly
		robot_strafe = self.smooth_robot_strafe
		
		# DEBUG: Log the actual values
		print(f"[NAVIGATOR-SPLINE-DEBUG] LatErr_raw={lateral_error:.1f}cm LatCorr={lateral_correction:.3f} RawStrafe={raw_robot_strafe:.3f} SmoothStrafe={robot_strafe:.3f}")
		
		# Calculate distance to end of spline
		distance_remaining = self.spline_total_distance - actual_distance
		
		# Velocity profiling - decelerate as we approach end
		decel_start_distance = 150.0  # cm from end (increased for better overshoot control)
		if distance_remaining < decel_start_distance:
			target_speed = self.max_move_speed * (distance_remaining / decel_start_distance)
			target_speed = max(self.min_drive_speed, target_speed)
		else:
			target_speed = self.max_move_speed
		
		# Ramp current speed
		if target_speed > self.current_drive_speed:
			self.current_drive_speed = min(target_speed, self.current_drive_speed + self.accel_rate)
		else:
			self.current_drive_speed = max(target_speed, self.current_drive_speed - self.decel_rate)
		
		drive_speed = self.current_drive_speed
		
		# Rotation PID: heading error toward desired heading
		heading_error = desired_heading - current_heading
		# Normalize to [-180, 180]
		if heading_error > 180:
			heading_error -= 360
		elif heading_error < -180:
			heading_error += 360
		
		# Normal heading correction - should be stable now that desired_heading is interpolated from waypoints
		if abs(heading_error) < 5.0:
			rotation_power = 0.0
			self.pid_rotate.reset()
		else:
			rotation_power = self.pid_rotate.calculate(heading_error)
			rotation_power = max(-self.max_rotation_speed, min(self.max_rotation_speed, rotation_power))
			# Enforce minimum power
			if 0 < abs(rotation_power) < 0.10:
				rotation_power = 0.10 if rotation_power > 0 else -0.10
		
		elapsed = time.time() - self.start_time
		
		# Debug output
		print(f"[NAVIGATOR-SPLINE] Dist={actual_distance:.1f}/{self.spline_total_distance:.1f}cm Remain={distance_remaining:.1f}cm Spd={drive_speed:.3f} HeadErr={heading_error:.1f}° LatErr={lateral_error:.1f}cm")
		
		# Send command to swerve drive
		self.drive.drive_swerve(robot_forward * drive_speed, robot_strafe * drive_speed, rotation_power)
		
		# Check if we've reached the end
		if distance_remaining < self.position_tolerance:
			self.drive.stop_all()
			self.is_active = False
			print(f"[NAVIGATOR-SPLINE] Reached end of spline!")
	
	def update(self):
		"""Call periodically (e.g., from robot loop) to execute navigation"""
		if not self.is_active:
			return
		
		# Pick up any live gain changes from SmartDashboard
		self._sync_gains()
		
		# Handle spline-following mode separately
		if self.use_spline and self.spline:
			self._update_spline()
			return
		
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
				# Velocity profile - two constraints, take the minimum:
				#   1. Decel toward the current waypoint (always - smooth approach to every wp)
				#   2. Decel toward the next mandatory stop (dwell or final waypoint)
				
				# Constraint 1: slow as we approach the current waypoint
				wp_decel_distance = 60.0  # cm - start braking this far from each waypoint
				if distance_to_target < wp_decel_distance:
					speed_from_wp = self.max_move_speed * (distance_to_target / wp_decel_distance)
					speed_from_wp = max(self.min_drive_speed, speed_from_wp)
				else:
					speed_from_wp = self.max_move_speed
				
				# Constraint 2: full stop at dwell/final waypoint
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
						speed_from_stop = self.max_move_speed * (dist_to_stop / self.decel_distance)
						speed_from_stop = max(0.0, speed_from_stop)
					else:
						speed_from_stop = self.max_move_speed
				else:
					speed_from_stop = self.max_move_speed
				
				# Use the more restrictive of the two speed limits
				target_speed = min(speed_from_wp, speed_from_stop)
				
				# Enforce min speed only when NOT in a decel-to-stop zone
				if speed_from_stop > self.min_drive_speed and target_speed < self.min_drive_speed:
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
					# Use 0.10 instead of 0.15 to allow smoother, smaller corrections
					if 0 < abs(rotation_power) < 0.10:
						rotation_power = 0.10 if rotation_power > 0 else -0.10
				
				# Calculate actual motor powers being sent to drive
				drive_fwd_power = robot_forward * drive_speed
				drive_str_power = robot_strafe * drive_speed
				
				# Debug: show velocity profiling details
				print(f"[NAVIGATOR-DEBUG] WP{self.current_waypoint_index + 1} Dist={distance_to_target:.1f}cm SpeedFromWP={speed_from_wp:.3f} SpeedFromStop={speed_from_stop:.3f} TargetSpd={target_speed:.3f} RampedSpd={drive_speed:.3f} FwdPwr={drive_fwd_power:.3f} StrPwr={drive_str_power:.3f} Rot={rotation_power:.3f}")
				
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
				self.start_time = time.time()
				self.current_drive_speed = 0.0
				self.pid_drive.reset()
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
