"""Encoder offset calibration management"""
import json
import os
from datetime import datetime
from swerve_config import OFFSET_FILE


class EncoderCalibration:
	"""Handle loading and saving encoder zero offsets and PID gains with battery level tracking"""
	
	def __init__(self):
		self.full_data = self.load_calibration()
		self.offsets = self.full_data.get("offsets", {})
		self.pid_gains = self.full_data.get("pid_gains", {})
		self.pid_tuning_history = self.full_data.get("pid_tuning_history", [])
		
		# Initialize regression data (will be calculated from history if it exists)
		self.pid_regression = {}
		if self.pid_tuning_history:
			self.update_gains_from_history()
	
	@staticmethod
	def get_calibration_path():
		"""Get the correct path for calibration file, supporting both robot and dev environments"""
		# Try RoboRIO path first
		if os.path.exists(OFFSET_FILE):
			return OFFSET_FILE
		
		# Fall back to local dev path (same directory as this script)
		script_dir = os.path.dirname(os.path.abspath(__file__))
		dev_path = os.path.join(script_dir, "encoder_offsets.json")
		
		# If neither exists, return the dev path (will be created there)
		return dev_path
	
	def load_calibration(self):
		"""Load calibration data (offsets and PID gains) from file"""
		defaults = {
			"offsets": {
				"front_right": 0.0,
				"rear_right": 0.0,
				"rear_left": 0.0,
				"front_left": 0.0
			},
			"pid_gains": {
				"kp": 0.003,
				"ki": 0.005,
				"kd": 0.0001
			},
			"pid_tuning_history": []
		}
		try:
			path = self.get_calibration_path()
			if os.path.exists(path):
				with open(path, "r") as f:
					data = json.load(f)
					# Handle legacy format (flat offsets) and new format (nested)
					if "offsets" in data:
						defaults.update(data)
					else:
						# Legacy format - just offsets
						defaults["offsets"].update(data)
					print(f"[LOAD] Loaded calibration from: {path}")
					print(f"       Offsets: {defaults['offsets']}")
					print(f"       PID: KP={defaults['pid_gains']['kp']}, KI={defaults['pid_gains']['ki']}, KD={defaults['pid_gains']['kd']}")
					if defaults['pid_tuning_history']:
						print(f"       Tuning history: {len(defaults['pid_tuning_history'])} entries")
					return defaults
		except Exception as e:
			print(f"[LOAD] Error loading calibration: {e}")
		
		print("[LOAD] No calibration file found, using defaults")
		return defaults
	
	def save_calibration(self):
		"""Save all calibration data to file"""
		try:
			self.full_data["offsets"] = self.offsets
			self.full_data["pid_gains"] = self.pid_gains
			self.full_data["pid_tuning_history"] = self.pid_tuning_history
			path = self.get_calibration_path()
			with open(path, "w") as f:
				json.dump(self.full_data, f, indent=2)
			print(f"[SAVE] Saved calibration to: {path}")
			print(f"       Offsets: {self.offsets}")
			print(f"       PID: KP={self.pid_gains['kp']:.6f}, KI={self.pid_gains['ki']:.6f}, KD={self.pid_gains['kd']:.6f}")
			print(f"       Tuning history: {len(self.pid_tuning_history)} entries")
			if self.pid_tuning_history:
				print(f"       Latest tuning: {self.pid_tuning_history[-1]}")
		except Exception as e:
			print(f"[SAVE] Failed to save calibration: {e}")
	
	def save_offsets(self):
		"""Save offsets to file (backward compatibility)"""
		self.save_calibration()
	
	def set_offset(self, wheel_name, offset):
		"""Set offset for a specific wheel"""
		self.offsets[wheel_name] = offset
	
	def get_offset(self, wheel_name):
		"""Get offset for a specific wheel"""
		return self.offsets.get(wheel_name, 0.0)
	
	def add_tuning_result(self, battery_voltage, wheel_gains):
		"""Add a new tuning result with per-wheel battery voltage and gains
		
		Args:
			battery_voltage: Battery voltage during tuning
			wheel_gains: Dict with wheel names as keys and {"kp", "ki", "kd"} dicts as values
		"""
		result = {
			"battery_voltage": round(battery_voltage, 2),
			"timestamp": datetime.now().isoformat(),
			"wheel_gains": wheel_gains
		}
		self.pid_tuning_history.append(result)
		
		# Print per-wheel gains
		for wheel_name, gains in wheel_gains.items():
			print(f"[TUNE] {wheel_name:12}: V={battery_voltage:.2f}V, KP={gains['kp']:.6f}, KI={gains['ki']:.6f}, KD={gains['kd']:.6f}")
		
		# Update current gains using interpolation
		self.update_gains_from_history()
	
	def update_gains_from_history(self):
		"""Calculate optimal PID gains per-wheel using linear regression across all tuning results"""
		if not self.pid_tuning_history:
			print("[TUNE] No tuning history, using defaults")
			return
		
		# Linear regression: fit per-wheel kp, ki, kd as functions of battery voltage
		wheel_names = ["front_left", "front_right", "rear_left", "rear_right"]
		
		# Check if we have per-wheel data (new format)
		has_per_wheel = isinstance(self.pid_tuning_history[0].get("wheel_gains"), dict)
		
		if has_per_wheel:
			self.pid_regression = {}
			for wheel_name in wheel_names:
				voltages = []
				kps = []
				kis = []
				kds = []
				
				for r in self.pid_tuning_history:
					if "wheel_gains" in r and wheel_name in r["wheel_gains"]:
						voltages.append(r["battery_voltage"])
						gains = r["wheel_gains"][wheel_name]
						kps.append(gains["kp"])
						kis.append(gains["ki"])
						kds.append(gains["kd"])
				
				if len(voltages) >= 1:
					kp_m, kp_b = self._linear_regression(voltages, kps)
					ki_m, ki_b = self._linear_regression(voltages, kis)
					kd_m, kd_b = self._linear_regression(voltages, kds)
					
					self.pid_regression[wheel_name] = {
						"kp": {"m": kp_m, "b": kp_b},
						"ki": {"m": ki_m, "b": ki_b},
						"kd": {"m": kd_m, "b": kd_b}
					}
		else:
			# Old format: single set of gains for all wheels
			voltages = [r["battery_voltage"] for r in self.pid_tuning_history]
			kps = [r["kp"] for r in self.pid_tuning_history]
			kis = [r["ki"] for r in self.pid_tuning_history]
			kds = [r["kd"] for r in self.pid_tuning_history]
			
			# Calculate best-fit line coefficients (y = mx + b)
			kp_m, kp_b = self._linear_regression(voltages, kps)
			ki_m, ki_b = self._linear_regression(voltages, kis)
			kd_m, kd_b = self._linear_regression(voltages, kds)
			
			print(f"\n[TUNE] Linear regression results (old format):")
			print(f"       KP = {kp_m:.6f} * V + {kp_b:.6f}")
			print(f"       KI = {ki_m:.6f} * V + {ki_b:.6f}")
			print(f"       KD = {kd_m:.6f} * V + {kd_b:.6f}")
			
			# Store regression coefficients for later interpolation (uniform for all wheels)
			self.pid_regression = {
				"kp": {"m": kp_m, "b": kp_b},
				"ki": {"m": ki_m, "b": ki_b},
				"kd": {"m": kd_m, "b": kd_b}
			}
		
		# Print per-wheel regressions if available
		if has_per_wheel:
			wheel_names = ["front_left", "front_right", "rear_left", "rear_right"]
			print(f"\n[TUNE] Per-wheel linear regression results:")
			for wheel_name in wheel_names:
				if wheel_name in self.pid_regression:
					kp = self.pid_regression[wheel_name]["kp"]
					ki = self.pid_regression[wheel_name]["ki"]
					kd = self.pid_regression[wheel_name]["kd"]
					print(f"       {wheel_name}:")
					print(f"         KP = {kp['m']:.6f} * V + {kp['b']:.6f}")
					print(f"         KI = {ki['m']:.6f} * V + {ki['b']:.6f}")
					print(f"         KD = {kd['m']:.6f} * V + {kd['b']:.6f}")
	
	@staticmethod
	def _linear_regression(x, y):
		"""Calculate linear regression coefficients (y = mx + b)"""
		n = len(x)
		if n < 2:
			return 0, y[0] if y else 0
		
		mean_x = sum(x) / n
		mean_y = sum(y) / n
		
		numerator = sum((x[i] - mean_x) * (y[i] - mean_y) for i in range(n))
		denominator = sum((x[i] - mean_x) ** 2 for i in range(n))
		
		if denominator == 0:
			return 0, mean_y
		
		m = numerator / denominator
		b = mean_y - m * mean_x
		return m, b
	
	def get_interpolated_gains(self, battery_voltage):
		"""Get PID gains interpolated for current battery voltage
		
		Returns per-wheel dict if per-wheel regression available, otherwise uniform dict
		"""
		if not hasattr(self, 'pid_regression') or not self.pid_regression:
			print(f"[INTERP] No regression data, using stored gains")
			return self.get_pid_gains()
		
		reg = self.pid_regression
		wheel_names = ["front_left", "front_right", "rear_left", "rear_right"]
		
		# Check if we have per-wheel regression data
		if wheel_names[0] in reg:
			# Per-wheel gains
			result = {}
			for wheel_name in wheel_names:
				if wheel_name in reg:
					kp = reg[wheel_name]["kp"]["m"] * battery_voltage + reg[wheel_name]["kp"]["b"]
					ki = reg[wheel_name]["ki"]["m"] * battery_voltage + reg[wheel_name]["ki"]["b"]
					kd = reg[wheel_name]["kd"]["m"] * battery_voltage + reg[wheel_name]["kd"]["b"]
					
					# Clamp to reasonable values
					kp = max(0.0001, min(0.1, kp))
					ki = max(0.0001, min(0.5, ki))
					kd = max(0.00001, min(0.01, kd))
					
					result[wheel_name] = {"kp": kp, "ki": ki, "kd": kd}
			return result
		else:
			# Uniform gains (old format or single-entry regression)
			kp = reg["kp"]["m"] * battery_voltage + reg["kp"]["b"]
			ki = reg["ki"]["m"] * battery_voltage + reg["ki"]["b"]
			kd = reg["kd"]["m"] * battery_voltage + reg["kd"]["b"]
			
			# Clamp to reasonable values
			kp = max(0.0001, min(0.1, kp))
			ki = max(0.0001, min(0.5, ki))
			kd = max(0.00001, min(0.01, kd))
			
			return {"kp": kp, "ki": ki, "kd": kd}
	
	def set_pid_gains(self, kp, ki, kd):
		"""Set PID gains for all wheels"""
		self.pid_gains["kp"] = kp
		self.pid_gains["ki"] = ki
		self.pid_gains["kd"] = kd
	
	def get_pid_gains(self):
		"""Get PID gains as dict"""
		return {
			"kp": self.pid_gains.get("kp", 0.003),
			"ki": self.pid_gains.get("ki", 0.005),
			"kd": self.pid_gains.get("kd", 0.0001)
		}
	
	def clear_tuning_history(self):
		"""Clear all tuning history and revert to defaults"""
		self.pid_tuning_history = []
		if hasattr(self, 'pid_regression'):
			delattr(self, 'pid_regression')
		# Reset to default gains
		self.pid_gains = {
			"kp": 0.003,
			"ki": 0.005,
			"kd": 0.0001
		}
		self.save_calibration()
		print("[CLEAR] Tuning history cleared, reverted to defaults")
