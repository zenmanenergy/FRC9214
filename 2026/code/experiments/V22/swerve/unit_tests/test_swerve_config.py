"""Unit tests for swerve.swerve_config constants and structure."""

from swerve import swerve_config as config


class TestWheelsStructure:

	EXPECTED_WHEEL_NAMES = {"front_right", "rear_right", "rear_left", "front_left"}
	REQUIRED_KEYS = {
		"drive_canid", "turn_canid", "encoder_dio", "button",
		"manual_offset", "rotation_angle", "position",
	}

	def test_has_exactly_four_wheels(self):
		assert set(config.WHEELS.keys()) == self.EXPECTED_WHEEL_NAMES

	def test_each_wheel_has_required_keys(self):
		for name, wheel in config.WHEELS.items():
			missing = self.REQUIRED_KEYS - wheel.keys()
			assert not missing, f"{name} is missing keys: {missing}"

	def test_position_has_x_and_y(self):
		for name, wheel in config.WHEELS.items():
			assert "x" in wheel["position"]
			assert "y" in wheel["position"]

	def test_rotation_angles_within_0_360(self):
		for name, wheel in config.WHEELS.items():
			assert 0 <= wheel["rotation_angle"] < 360, name

	def test_drive_canids_are_unique(self):
		drive_ids = [w["drive_canid"] for w in config.WHEELS.values()]
		assert len(drive_ids) == len(set(drive_ids))

	def test_turn_canids_are_unique(self):
		turn_ids = [w["turn_canid"] for w in config.WHEELS.values()]
		assert len(turn_ids) == len(set(turn_ids))

	def test_drive_and_turn_canids_do_not_overlap(self):
		drive_ids = {w["drive_canid"] for w in config.WHEELS.values()}
		turn_ids = {w["turn_canid"] for w in config.WHEELS.values()}
		assert drive_ids.isdisjoint(turn_ids)

	def test_encoder_dio_ports_are_unique(self):
		dio_ports = [w["encoder_dio"] for w in config.WHEELS.values()]
		assert len(dio_ports) == len(set(dio_ports))


class TestControlConstants:

	def test_motor_scales_within_valid_range(self):
		assert 0.0 < config.MOTOR_SCALE_ALIGN <= 1.0
		assert 0.0 < config.MOTOR_SCALE_TELEOP <= 1.0

	def test_alignment_tolerances_are_positive(self):
		assert config.ALIGN_TOLERANCE > 0
		assert config.ALIGN_TIMEOUT > 0
		assert config.DRIVE_ANGLE_TOLERANCE > 0

	def test_robot_dimensions_are_positive(self):
		assert config.ROBOT_TRACKWIDTH_CM > 0
		assert config.ROBOT_WHEELBASE_CM > 0

	def test_offset_file_is_a_nonempty_path(self):
		assert isinstance(config.OFFSET_FILE, str)
		assert len(config.OFFSET_FILE) > 0
