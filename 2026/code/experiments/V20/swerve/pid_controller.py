"""PID Controller for smooth wheel alignment"""
import wpilib
import time


class PIDController:
	"""A simple PID controller for motor control"""
	
	def __init__(self, kp, ki, kd, name="PID"):
		"""
		Initialize PID controller
		
		Args:
			kp: Proportional gain
			ki: Integral gain (accumulation of error over time)
			kd: Derivative gain (rate of change of error)
			name: Name for debugging
		"""
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.name = name
		
		self.integral_sum = 0.0
		self.last_error = 0.0
		self.last_time = wpilib.Timer.getFPGATimestamp()
		self.max_integral = 0.5  # Max integral accumulation to prevent windup
		
		# Store last calculated terms for debugging
		self.last_p_term = 0.0
		self.last_i_term = 0.0
		self.last_d_term = 0.0
		
	def calculate(self, error, current_time=None):
		"""
		Calculate PID output
		
		Args:
			error: Current error (setpoint - actual)
			current_time: Current time (optional, uses FPGA time if not provided)
			
		Returns:
			PID output value
		"""
		if current_time is None:
			current_time = wpilib.Timer.getFPGATimestamp()
		
		dt = current_time - self.last_time
		
		# Prevent division by zero and skip if dt is zero
		if dt <= 0:
			return 0.0
		
		# Proportional term
		p_term = self.kp * error
		
		# Integral term with anti-windup
		self.integral_sum += error * dt
		self.integral_sum = max(-self.max_integral, min(self.max_integral, self.integral_sum))
		i_term = self.ki * self.integral_sum
		
		# Derivative term
		if dt > 0:
			error_rate = (error - self.last_error) / dt
		else:
			error_rate = 0
		d_term = self.kd * error_rate
		
		# Store terms for debugging
		self.last_p_term = p_term
		self.last_i_term = i_term
		self.last_d_term = d_term
		
		# Update state
		self.last_error = error
		self.last_time = current_time
		
		# Calculate output
		output = p_term + i_term + d_term
		
		return output
	
	def reset(self):
		"""Reset PID controller state"""
		self.integral_sum = 0.0
		self.last_error = 0.0
		self.last_time = wpilib.Timer.getFPGATimestamp()
	
	def set_gains(self, kp, ki, kd):
		"""Update PID gains"""
		self.kp = kp
		self.ki = ki
		self.kd = kd
	
	def autotune(self, get_error_func, set_output_func, max_power=1.0, duration_seconds=15.0, target_cycles=3):
		"""
		Autotune PID gains using Ziegler-Nichols relay method.
		
		Applies bang-bang control and measures oscillation to calculate optimal gains.
		
		Args:
			get_error_func: Callable that returns current error value
			set_output_func: Callable(power) that applies control output
			max_power: Maximum relay power amplitude (default 1.0)
			duration_seconds: Max time to run autotune (default 15s)
			target_cycles: Number of oscillation cycles to measure (default 3)
		
		Returns:
			dict with keys: 'kp', 'ki', 'kd', 'success', 'period', 'amplitude', 'message'
		"""
		print(f"\n[{self.name}-AUTOTUNE] Starting... (max {duration_seconds}s, {target_cycles} cycles)")
		
		start_time = time.time()
		oscillations = []  # Track zero crossings and timings
		last_error = 0.0
		relay_power = max_power
		cycle_count = 0
		
		try:
			while time.time() - start_time < duration_seconds and cycle_count < target_cycles:
				current_error = get_error_func()
				
				# Toggle relay at zero crossing (sign change)
				if last_error * current_error < 0:  # Sign change = zero crossing
					zero_cross_time = time.time() - start_time
					oscillations.append({
						'time': zero_cross_time,
						'error': current_error,
						'amplitude': abs(current_error)
					})
					relay_power = -relay_power
					cycle_count += 1
					print(f"[{self.name}-AUTOTUNE] Cycle {cycle_count}: t={zero_cross_time:.3f}s, err={current_error:.2f}")
				
				set_output_func(relay_power)
				last_error = current_error
				time.sleep(0.02)  # 50Hz update rate
		
		except Exception as e:
			set_output_func(0.0)
			return {
				'success': False,
				'kp': self.kp, 'ki': self.ki, 'kd': self.kd,
				'message': f'Autotune failed: {str(e)}'
			}
		
		set_output_func(0.0)
		
		# Analyze results
		if len(oscillations) < 2:
			return {
				'success': False,
				'kp': self.kp, 'ki': self.ki, 'kd': self.kd,
				'message': f'Not enough oscillations detected ({len(oscillations)}). System may be too damped or gains too low.'
			}
		
		# Calculate ultimate period (time between oscillations * 2)
		time_diffs = [oscillations[i+1]['time'] - oscillations[i]['time'] for i in range(len(oscillations)-1)]
		ultimate_period = sum(time_diffs) / len(time_diffs) * 2.0
		
		# Calculate average amplitude
		amplitudes = [osc['amplitude'] for osc in oscillations]
		ultimate_amplitude = sum(amplitudes) / len(amplitudes)
		
		# Ziegler-Nichols formulas for ultimate gain method
		# Ku = 4*max_relay_amplitude / (pi * ultimate_amplitude)
		# Pu = ultimate_period
		ku = (4.0 * max_power) / (3.14159 * ultimate_amplitude)
		pu = ultimate_period
		
		# Classic ZN tuning (for moderate overshoot)
		kp_new = 0.6 * ku
		ki_new = 1.2 * ku / pu
		kd_new = 0.075 * ku * pu
		
		# Store new gains
		self.kp = kp_new
		self.ki = ki_new
		self.kd = kd_new
		self.reset()
		
		result = {
			'success': True,
			'kp': kp_new,
			'ki': ki_new,
			'kd': kd_new,
			'period': ultimate_period,
			'amplitude': ultimate_amplitude,
			'ku': ku,
			'pu': pu,
			'cycles': len(oscillations),
			'message': f'Autotune complete: kp={kp_new:.6f}, ki={ki_new:.6f}, kd={kd_new:.6f}'
		}
		
		print(f"[{self.name}-AUTOTUNE] {result['message']}")
		print(f"[{self.name}-AUTOTUNE] Ultimate period={ultimate_period:.3f}s, amplitude={ultimate_amplitude:.2f}")
		
		return result
