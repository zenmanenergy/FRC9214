"""Encoder offset calibration management"""
import json
import os
from swerve_config import OFFSET_FILE


class EncoderCalibration:
	"""Handle loading and saving encoder zero offsets"""
	
	def __init__(self):
		self.offsets = self.load_offsets()
	
	def load_offsets(self):
		"""Load zero offsets from file"""
		defaults = {
			"front_right": 0.0,
			"rear_right": 0.0,
			"rear_left": 0.0,
			"front_left": 0.0
		}
		try:
			if os.path.exists(OFFSET_FILE):
				with open(OFFSET_FILE, "r") as f:
					data = json.load(f)
					defaults.update(data)
					print(f"[LOAD] Loaded offsets: {defaults}")
					return defaults
		except Exception as e:
			print(f"[LOAD] Error loading offsets: {e}")
		
		print("[LOAD] No offset file found, starting with all zeros")
		return defaults
	
	def save_offsets(self):
		"""Save all zero offsets to file"""
		try:
			with open(OFFSET_FILE, "w") as f:
				json.dump(self.offsets, f)
			print(f"[SAVE] Saved offsets: {self.offsets}")
		except Exception as e:
			print(f"[SAVE] Failed to save offsets: {e}")
	
	def set_offset(self, wheel_name, offset):
		"""Set offset for a specific wheel"""
		self.offsets[wheel_name] = offset
	
	def get_offset(self, wheel_name):
		"""Get offset for a specific wheel"""
		return self.offsets.get(wheel_name, 0.0)
