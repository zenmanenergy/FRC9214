class ArmPresets:
	def __init__(self, arm, table):
		self.arm = arm
		self.table = table

		# Define preset positions
		self.presets = {
			"load": {"elevator": 40, "shoulder": 140, "wrist": 45},
			"stow": {"elevator": 10, "shoulder": 90, "wrist": 0},
			"score_high": {"elevator": 80, "shoulder": 160, "wrist": 30},
			"score_low": {"elevator": 20, "shoulder": 120, "wrist": 10},
			"pickup": {"elevator": 5, "shoulder": 60, "wrist": -10},
		}

	def gotopreset(self, name):
		"""Sets the target positions for a preset without blocking."""
		if name not in self.presets:
			print(f"Preset '{name}' not found.")
			return

		# Set target positions inside Arm
		preset = self.presets[name]
		self.arm.set_target_positions(
			preset["elevator"], preset["shoulder"], preset["wrist"]
		)

		# Send target positions to NetworkTables for reference
		self.table.putNumber("target_elevator", preset["elevator"])
		self.table.putNumber("target_shoulder", preset["shoulder"])
		self.table.putNumber("target_wrist", preset["wrist"])

		print(f"Set target for preset '{name}': Elevator={preset['elevator']}cm, Shoulder={preset['shoulder']}°, Wrist={preset['wrist']}°")
