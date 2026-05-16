import wpilib
from wpilib import SmartDashboard, RobotController
from .swerve_wheel import SwerveWheel
from .encoder_calibration import EncoderCalibration
from .pid_controller import PIDController
from .swerve_odometry import SwerveOdometry
from .swerve_imu import SwerveIMU
from .swerve_tune import SwerveTuner
from . import swerve_config as config
import math


class SwerveDrive:
	
	def __init__(self):
		
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
	
	def stop_all(self):
		for wheel in self.wheels.values():
			wheel.stop()
		self.wheel_alignment_state.clear()
		self.movement_state = "idle"
		self.per_wheel_previous_power = {name: 0.0 for name in self.wheels.keys()}
	
	def _apply_smooth_acceleration(self, wheel_name, target_power):
		previous_power = self.per_wheel_previous_power.get(wheel_name, 0.0)
		
		power_diff = target_power - previous_power
		ramped_power = previous_power + (power_diff * self.power_ramp_rate)
		
		self.per_wheel_previous_power[wheel_name] = ramped_power
		
		return ramped_power
	
	def is_moving(self):
		return self.movement_state in ["moving", "aligning"]
	
	def is_aligning(self):
		return len(self.wheel_alignment_state) > 0
	
	def get_movement_state(self):
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
	
	def drive_straight(self, speed, target_angle=0.0):
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
	
	def drive_to_heading(self, target_angle):
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
	
	def drive_for_distance(self, speed, target_distance_cm, target_angle=0.0):
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
	
	def drive_swerve(self, forward, strafe, rotate):
		deadzone = 0.1
		forward = forward if abs(forward) >= deadzone else 0.0
		strafe = strafe if abs(strafe) >= deadzone else 0.0
		rotate = rotate if abs(rotate) >= deadzone else 0.0
		
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
			
			if angle_error <= config.DRIVE_ANGLE_TOLERANCE:
				drive_power = -target_speed * config.MOTOR_SCALE_TELEOP
				ramped_power = self._apply_smooth_acceleration(wheel_name, drive_power)
				wheel.set_drive_power(ramped_power)
				if abs(target_speed) > 0.01:
					self.movement_state = "moving"
					self.last_move_time = wpilib.Timer.getFPGATimestamp()
			else:
				ramped_power = self._apply_smooth_acceleration(wheel_name, 0.0)
				wheel.set_drive_power(ramped_power)
				if len(self.wheel_alignment_state) > 0:
					self.movement_state = "aligning"
		
		self.update_motor_currents()
	
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
		
		drive_power = -abs(rotation_input) * config.MOTOR_SCALE_TELEOP
		
		for wheel_name in config.WHEELS.keys():
			base_angle = config.WHEELS[wheel_name]["rotation_angle"]
			if rotation_input <= 0:
				target_angle = (base_angle + 180) % 360
			else:
				target_angle = base_angle
			
			wheel_data = config.WHEELS[wheel_name]
			target_angle = (target_angle + wheel_data["manual_offset"]) % 360
			
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
	
	def start_autotune(self):
		self.tuner.start()
	
