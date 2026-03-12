"""PID Controller for smooth wheel alignment"""
import wpilib


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
