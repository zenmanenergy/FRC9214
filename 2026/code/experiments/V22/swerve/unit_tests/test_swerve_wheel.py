"""Unit tests for swerve.swerve_wheel.SwerveWheel.

The wheel is constructed once for the whole module (real DIO channel is a
HAL-simulated resource that cannot be re-allocated without closing it, and
wpilib.DutyCycleEncoder has no close()). Each test replaces drive_motor /
turn_motor with FakeMotor doubles so behavior is deterministic regardless of
the installed rev SDK version, and uses DutyCycleEncoderSim (which wraps the
already-allocated encoder, not a new channel) to control the simulated angle.
"""

import pytest
import wpilib
from wpilib.simulation import DutyCycleEncoderSim

from swerve.swerve_wheel import SwerveWheel
from swerve.unit_tests.fakes import FakeMotor

# Use CAN IDs / DIO channel that don't collide with real config.WHEELS values
# or with any other test module's hardware allocations.
TEST_DRIVE_CANID = 90
TEST_TURN_CANID = 91
TEST_ENCODER_DIO = 7


@pytest.fixture(scope="module")
def wheel():
	return SwerveWheel(
		"front_right",
		drive_canid=TEST_DRIVE_CANID,
		turn_canid=TEST_TURN_CANID,
		encoder_dio=TEST_ENCODER_DIO,
		manual_offset=0.0,
	)


@pytest.fixture(autouse=True)
def fresh_motors(wheel):
	"""Replace real motors with fakes before each test, and reset wheel state."""
	wheel.drive_motor = FakeMotor()
	wheel.turn_motor = FakeMotor()
	wheel.offset = 0.0
	wheel.current_drive_power = 0.0
	wheel._turn_motor_error_logged = False
	yield


@pytest.fixture
def encoder_sim(wheel):
	sim = DutyCycleEncoderSim(wheel.encoder)
	sim.set(0.0)
	yield sim
	sim.set(0.0)


class TestAngle:

	def test_get_raw_angle_reflects_encoder_fraction(self, wheel, encoder_sim):
		encoder_sim.set(0.25)
		assert wheel.get_raw_angle() == pytest.approx(90.0)

	def test_get_angle_applies_offset(self, wheel, encoder_sim):
		encoder_sim.set(0.5)  # raw = 180
		wheel.offset = 30.0
		assert wheel.get_angle() == 150

	def test_get_angle_wraps_into_0_360(self, wheel, encoder_sim):
		encoder_sim.set(0.0)  # raw = 0
		wheel.offset = 10.0
		assert wheel.get_angle() == 350

	def test_set_and_get_zero_offset(self, wheel):
		wheel.set_zero_offset(123.4)
		assert wheel.get_zero_offset() == pytest.approx(123.4)
		assert wheel.offset == pytest.approx(123.4)


class TestDrivePower:

	@pytest.mark.parametrize("wheel_name_power", [0.5, -0.5, 0.0])
	def test_set_drive_power_tracks_current_power(self, wheel, wheel_name_power):
		wheel.set_drive_power(wheel_name_power)
		assert wheel.get_drive_power() == wheel_name_power

	def test_set_drive_power_inverts_for_front_right(self, wheel):
		# front_right inverts drive direction in set_drive_power
		wheel.set_drive_power(0.7)
		assert wheel.drive_motor.power == pytest.approx(-0.7)


class TestTurnPower:

	def test_set_turn_power_sends_inverted_command(self, wheel):
		wheel.set_turn_power(0.3)
		assert wheel.turn_motor.power == pytest.approx(-0.3)

	def test_set_turn_power_noop_and_logs_once_when_motor_missing(self, wheel):
		wheel.turn_motor = None
		wheel.set_turn_power(0.5)  # Should not raise
		assert wheel._turn_motor_error_logged is True


class TestStop:

	def test_stop_zeros_both_motors(self, wheel):
		wheel.drive_motor.power = 0.9
		wheel.turn_motor.power = 0.9

		wheel.stop()

		assert wheel.drive_motor.power == 0.0
		assert wheel.turn_motor.power == 0.0

	def test_stop_handles_missing_motors_gracefully(self, wheel):
		wheel.drive_motor = None
		wheel.turn_motor = None
		wheel.stop()  # Should not raise


class TestDriveEncoderReadouts:

	def test_get_drive_position_reads_fake_encoder(self, wheel):
		wheel.drive_motor.getEncoder().position = 3.5
		assert wheel.get_drive_position() == pytest.approx(3.5)

	def test_get_drive_position_defaults_zero_without_motor(self, wheel):
		wheel.drive_motor = None
		assert wheel.get_drive_position() == 0.0

	def test_get_drive_velocity_reads_fake_encoder(self, wheel):
		wheel.drive_motor.getEncoder().velocity = 42.0
		assert wheel.get_drive_velocity() == pytest.approx(42.0)

	def test_get_drive_velocity_defaults_zero_without_motor(self, wheel):
		wheel.drive_motor = None
		assert wheel.get_drive_velocity() == 0.0

	def test_get_drive_distance_converts_rotations_to_cm(self, wheel):
		wheel.drive_motor.getEncoder().position = 1.0
		# front_right inverts position sign in get_drive_distance
		import math
		expected = -1.0 * math.pi * 10.16
		assert wheel.get_drive_distance() == pytest.approx(expected)

	def test_get_drive_distance_custom_wheel_diameter(self, wheel):
		wheel.drive_motor.getEncoder().position = 1.0
		import math
		expected = -1.0 * math.pi * 5.0
		assert wheel.get_drive_distance(wheel_diameter_cm=5.0) == pytest.approx(expected)
