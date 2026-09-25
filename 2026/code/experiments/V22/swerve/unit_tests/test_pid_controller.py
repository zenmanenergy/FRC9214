"""Unit tests for swerve.pid_controller.PIDController."""

import pytest

from swerve.pid_controller import PIDController


class TestCalculate:

	def test_zero_dt_returns_zero(self):
		pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
		pid.last_time = 100.0
		output = pid.calculate(error=5.0, current_time=100.0)
		assert output == 0.0

	def test_pure_proportional(self):
		pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
		pid.last_time = 0.0
		output = pid.calculate(error=3.0, current_time=1.0)
		assert output == pytest.approx(6.0)
		assert pid.last_p_term == pytest.approx(6.0)
		assert pid.last_i_term == pytest.approx(0.0)
		assert pid.last_d_term == pytest.approx(0.0)

	def test_pure_integral_accumulates_over_time(self):
		pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
		pid.last_time = 0.0
		pid.calculate(error=0.1, current_time=1.0)  # integral_sum = 0.1
		output = pid.calculate(error=0.1, current_time=2.0)  # integral_sum = 0.2 (below max_integral clamp)
		assert output == pytest.approx(0.2)

	def test_integral_anti_windup_clamps(self):
		pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
		pid.max_integral = 0.5
		pid.last_time = 0.0
		# Huge error/time should saturate the integral at max_integral
		output = pid.calculate(error=1000.0, current_time=10.0)
		assert output == pytest.approx(0.5)

	def test_pure_derivative(self):
		pid = PIDController(kp=0.0, ki=0.0, kd=1.0)
		pid.last_time = 0.0
		pid.last_error = 0.0
		output = pid.calculate(error=10.0, current_time=1.0)
		# error_rate = (10 - 0) / 1 = 10
		assert output == pytest.approx(10.0)

	def test_updates_last_error_and_time(self):
		pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
		pid.last_time = 0.0
		pid.calculate(error=7.0, current_time=2.0)
		assert pid.last_error == 7.0
		assert pid.last_time == 2.0

	def test_uses_fpga_timestamp_when_current_time_omitted(self):
		pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
		pid.last_time = 0.0
		output = pid.calculate(error=1.0)
		# Real FPGA time is >> 0, so dt is large and output should just be kp*error
		assert output == pytest.approx(1.0)


class TestResetAndGains:

	def test_reset_clears_state(self):
		pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
		pid.last_time = 0.0
		pid.calculate(error=5.0, current_time=1.0)
		assert pid.integral_sum != 0.0

		pid.reset()
		assert pid.integral_sum == 0.0
		assert pid.last_error == 0.0

	def test_set_gains_updates_values(self):
		pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
		pid.set_gains(kp=0.1, ki=0.2, kd=0.3)
		assert pid.kp == pytest.approx(0.1)
		assert pid.ki == pytest.approx(0.2)
		assert pid.kd == pytest.approx(0.3)


class TestAutotune:

	def test_autotune_reports_failure_without_oscillation(self):
		pid = PIDController(kp=0.01, ki=0.0, kd=0.0, name="test")
		outputs = []

		def get_error():
			return 5.0  # Constant error, never crosses zero -> no oscillation

		def set_output(power):
			outputs.append(power)

		result = pid.autotune(
			get_error_func=get_error,
			set_output_func=set_output,
			max_power=0.5,
			duration_seconds=0.2,
			target_cycles=2,
		)

		assert result["success"] is False
		assert "message" in result

	def test_autotune_succeeds_with_simulated_oscillation(self):
		pid = PIDController(kp=0.01, ki=0.0, kd=0.0, name="test")

		# Deterministic error sequence that flips sign every call, forcing
		# quick zero-crossings so the relay method converges in a couple cycles.
		errors = iter([1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0])

		def get_error():
			return next(errors, -1.0)

		def set_output(power):
			pass

		result = pid.autotune(
			get_error_func=get_error,
			set_output_func=set_output,
			max_power=1.0,
			duration_seconds=2.0,
			target_cycles=3,
		)

		assert result["success"] is True
		for key in ("kp", "ki", "kd", "period", "amplitude"):
			assert key in result
		# Gains should have been applied to the controller itself
		assert pid.kp == pytest.approx(result["kp"])

	def test_autotune_handles_exception_in_callbacks(self):
		pid = PIDController(kp=0.01, ki=0.0, kd=0.0, name="test")

		def get_error():
			raise RuntimeError("sensor failure")

		def set_output(power):
			pass

		result = pid.autotune(
			get_error_func=get_error,
			set_output_func=set_output,
			duration_seconds=0.2,
			target_cycles=1,
		)

		assert result["success"] is False
		assert "Autotune failed" in result["message"]
