"""Robot Simulator - Simulates robot movement without hardware"""
import math
import time


class SimulatedWheel:
	"""Simulated swerve wheel"""
	def __init__(self, name):
		self.name = name
		self.angle = 0
		self.power = 0
		self.position = 0  # in rotations
	
	def set_angle(self, angle):
		self.angle = angle % 360
	
	def set_power(self, power):
		self.power = max(-1.0, min(1.0, power))
	
	def get_angle(self):
		return self.angle
	
	def get_power(self):
		return self.power
	
	def get_position(self):
		return self.position
	
	def update(self, dt):
		"""Update position based on power (dt in seconds)"""
		# Simulate motor velocity: power * max_rpm / 60 * dt
		# 1 rotation = 1 wheel rotation = circumference distance
		max_rotations_per_sec = 0.2  # Roughly 200 RPM
		self.position += self.power * max_rotations_per_sec * dt


class SimulatedOdometry:
	"""Simulated odometry system"""
	
	WHEEL_DIAMETER_CM = 10.16  # 4 inch wheel
	WHEEL_CIRCUMFERENCE_CM = math.pi * 10.16
	DRIVE_GEAR_RATIO = 6.3
	
	def __init__(self, wheels):
		self.wheels = wheels
		self._x = 0.0
		self._y = 0.0
		self._heading = 0.0
		self._distance = 0.0
		self._prev_positions = {name: 0 for name in wheels.keys()}
	
	def update(self, dt):
		"""Update pose based on wheel positions"""
		for name, wheel in self.wheels.items():
			pos = wheel.get_position()
			delta_rotations = pos - self._prev_positions[name]
			self._prev_positions[name] = pos
			
			# Distance traveled by this wheel
			distance_cm = delta_rotations * self.WHEEL_CIRCUMFERENCE_CM
			
			# Convert wheel rotation to robot movement (simplified 4-wheel average)
			avg_distance = distance_cm / 4.0  # Average across all wheels
			self._distance += abs(distance_cm / 4.0)
		
		# Simple heading update (would be from gyro in real robot)
		# For simulation, assume robot doesn't rotate much
		pass
	
	def set_position(self, x, y):
		self._x = x
		self._y = y
	
	def set_heading(self, heading):
		self._heading = heading % 360
	
	def get_position(self):
		return (self._x, self._y)
	
	def get_x(self):
		return self._x
	
	def get_y(self):
		return self._y
	
	def get_heading(self):
		return self._heading
	
	def get_total_distance(self):
		return self._distance


class RobotSimulator:
	"""Complete robot simulator for testing without hardware"""
	
	def __init__(self):
		"""Initialize simulated robot"""
		self.wheels = {
			"front_right": SimulatedWheel("front_right"),
			"rear_right": SimulatedWheel("rear_right"),
			"rear_left": SimulatedWheel("rear_left"),
			"front_left": SimulatedWheel("front_left")
		}
		self.odometry = SimulatedOdometry(self.wheels)
		
		# Physics
		self.last_update = time.time()
		self.dt = 0.02  # 50Hz
		
		# Movement state
		self.target_angles = {name: 0 for name in self.wheels.keys()}
		self.target_powers = {name: 0 for name in self.wheels.keys()}
		self.rotation_speed = 180  # deg/sec
		self.acceleration = 2.0  # power/sec
	
	def set_wheel_angle(self, wheel_name, angle):
		"""Set target angle for a wheel"""
		if wheel_name in self.wheels:
			self.target_angles[wheel_name] = angle % 360
	
	def set_wheel_power(self, wheel_name, power):
		"""Set target power for a wheel"""
		if wheel_name in self.wheels:
			self.target_powers[wheel_name] = max(-1.0, min(1.0, power))
	
	def set_all_wheels(self, angles, powers):
		"""Set all wheels at once"""
		for name in self.wheels.keys():
			if name in angles:
				self.set_wheel_angle(name, angles[name])
			if name in powers:
				self.set_wheel_power(name, powers[name])
	
	def stop_all(self):
		"""Stop all wheels"""
		for name in self.wheels.keys():
			self.set_wheel_power(name, 0)
	
	def update(self):
		"""Update simulation (call at ~50Hz)"""
		current_time = time.time()
		dt = min(0.05, current_time - self.last_update)  # Cap dt
		self.last_update = current_time
		
		# Update wheel angles (smooth rotation)
		for name, wheel in self.wheels.items():
			current_angle = wheel.get_angle()
			target_angle = self.target_angles[name]
			
			# Calculate shortest rotation
			angle_diff = target_angle - current_angle
			if angle_diff > 180:
				angle_diff -= 360
			elif angle_diff < -180:
				angle_diff += 360
			
			# Rotate at maximum speed, capped
			max_rotation = self.rotation_speed * dt
			if abs(angle_diff) > max_rotation:
				new_angle = current_angle + (max_rotation if angle_diff > 0 else -max_rotation)
			else:
				new_angle = target_angle
			
			wheel.set_angle(new_angle % 360)
		
		# Update wheel powers (smooth acceleration)
		for name, wheel in self.wheels.items():
			current_power = wheel.get_power()
			target_power = self.target_powers[name]
			
			power_diff = target_power - current_power
			max_accel = self.acceleration * dt
			
			if abs(power_diff) > max_accel:
				new_power = current_power + (max_accel if power_diff > 0 else -max_accel)
			else:
				new_power = target_power
			
			wheel.set_power(new_power)
		
		# Update positions and odometry
		for wheel in self.wheels.values():
			wheel.update(dt)
		
		self.odometry.update(dt)
		
		# Simplified movement: average wheel powers and angles
		avg_power = sum(wheel.get_power() for wheel in self.wheels.values()) / 4.0
		avg_angle = sum(wheel.get_angle() for wheel in self.wheels.values()) / 4.0
		
		# Update position
		if abs(avg_power) > 0.01:
			distance_per_sec = avg_power * 50  # cm/sec
			rad = math.radians(avg_angle)
			dx = distance_per_sec * math.cos(rad) * self.dt
			dy = distance_per_sec * math.sin(rad) * self.dt
			
			current_x, current_y = self.odometry.get_position()
			self.odometry.set_position(current_x + dx, current_y + dy)
	
	def get_state(self):
		"""Get full robot state for dashboard"""
		angles = {name: wheel.get_angle() for name, wheel in self.wheels.items()}
		powers = {name: wheel.get_power() for name, wheel in self.wheels.items()}
		
		return {
			"angles": angles,
			"powers": powers,
			"odometry": {
				"x": round(self.odometry.get_x(), 2),
				"y": round(self.odometry.get_y(), 2),
				"heading": round(self.odometry.get_heading(), 1),
				"distance": round(self.odometry.get_total_distance(), 2)
			}
		}
