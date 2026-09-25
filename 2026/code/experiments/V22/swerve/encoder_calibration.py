"""Encoder offset calibration and PID tuning result management.

Persistent storage and interpolation of:
  - Per-wheel encoder zero offsets
  - Per-wheel PID gains with battery voltage correlation
  - Tuning history for trend analysis

Calibration data is stored in JSON format on the RoboRIO's file system.
Gains are interpolated based on current battery voltage for adaptive control.

Example:
    cal = EncoderCalibration()
    cal.set_offset('front_right', 45.0)  # Mark wheel as pointing at 45°
    cal.save_calibration()
    
    # Load and interpolate gains for current battery voltage
    gains = cal.get_interpolated_gains(12.0)  # 12V battery
    print(f"KP={gains['front_left']['kp']:.6f}")
"""

from typing import Dict, Optional, List, Tuple
import json
import os
from datetime import datetime
from .swerve_config import OFFSET_FILE

WHEEL_NAMES = ["front_left", "front_right", "rear_left", "rear_right"]


class EncoderCalibration:
	"""Handle loading and saving encoder zero offsets and PID gains with battery level tracking"""
	
	def __init__(self) -> None:
		self.full_data = self.load_calibration()
		self.offsets = self.full_data.get("offsets", {})
		self.pid_gains = self.full_data.get("pid_gains", {})
		self.pid_tuning_history = self.full_data.get("pid_tuning_history", [])
		self.navigator_rotation_gains = self.full_data.get("navigator_rotation_gains", {})
		self.rotation_calibration = self.full_data.get("rotation_calibration", self._default_rotation_calibration())
		self.translation_calibration = self.full_data.get("translation_calibration", self._default_translation_calibration())
		
		# Initialize regression data (will be calculated from history if it exists)
		self.pid_regression = {}
		if self.pid_tuning_history:
			self.update_gains_from_history()
	
	@staticmethod
	def get_calibration_path() -> str:
		"""Get the calibration file path on RoboRIO"""
		return OFFSET_FILE
	
	def load_calibration(self) -> Dict:
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
			"navigator_rotation_gains": {
				"kp": 0.004,
				"ki": 0.0,
				"kd": 0.0001
			},
			"pid_tuning_history": [],
			"rotation_calibration": self._default_rotation_calibration(),
			"translation_calibration": self._default_translation_calibration()
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
					print(f"       Wheel PID: KP={defaults['pid_gains']['kp']}, KI={defaults['pid_gains']['ki']}, KD={defaults['pid_gains']['kd']}")
					if "navigator_rotation_gains" in defaults:
						gains = defaults['navigator_rotation_gains']
						print(f"       Navigator Rotation: KP={gains.get('kp', 0.004)}, KI={gains.get('ki', 0.0)}, KD={gains.get('kd', 0.0001)}")
					if defaults['pid_tuning_history']:
						print(f"       Tuning history: {len(defaults['pid_tuning_history'])} entries")
					return defaults
		except Exception as e:
			print(f"[LOAD] Error loading calibration: {e}")
		
		print("[LOAD] No calibration file found, using defaults")
		return defaults
	
	def save_calibration(self) -> None:
		"""Save all calibration data to file"""
		try:
			self.full_data["offsets"] = self.offsets
			self.full_data["pid_gains"] = self.pid_gains
			self.full_data["navigator_rotation_gains"] = self.navigator_rotation_gains
			self.full_data["pid_tuning_history"] = self.pid_tuning_history
			self.full_data["rotation_calibration"] = self.rotation_calibration
			self.full_data["translation_calibration"] = self.translation_calibration
			path = self.get_calibration_path()
			with open(path, "w") as f:
				json.dump(self.full_data, f, indent=2)
			print(f"[SAVE] Saved calibration to: {path}")
			print(f"[SAVE] File contents being written:")
			print(json.dumps(self.full_data, indent=2))
			print(f"       Offsets: {self.offsets}")
			print(f"       Wheel PID: KP={self.pid_gains['kp']:.6f}, KI={self.pid_gains['ki']:.6f}, KD={self.pid_gains['kd']:.6f}")
			if self.navigator_rotation_gains:
				gains = self.navigator_rotation_gains
				print(f"       Navigator Rotation: KP={gains.get('kp', 0):.6f}, KI={gains.get('ki', 0):.6f}, KD={gains.get('kd', 0):.6f}")
			print(f"       Tuning history: {len(self.pid_tuning_history)} entries")
			if self.pid_tuning_history:
				print(f"       Latest tuning: {self.pid_tuning_history[-1]}")
		except Exception as e:
			print(f"[SAVE] Failed to save calibration: {e}")
	
	def save_offsets(self) -> None:
		"""Save offsets to file (backward compatibility)."""
		self.save_calibration()
	
	def set_offset(self, wheel_name: str, offset: float) -> None:
		"""Set offset for a specific wheel."""
		self.offsets[wheel_name] = offset
	
	def get_offset(self, wheel_name: str) -> float:
		"""Get offset for a specific wheel in degrees."""
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
			
			self.pid_regression = {
				"kp": {"m": kp_m, "b": kp_b},
				"ki": {"m": ki_m, "b": ki_b},
				"kd": {"m": kd_m, "b": kd_b}
			}
		
		if has_per_wheel:
			wheel_names = ["front_left", "front_right", "rear_left", "rear_right"]
			for wheel_name in wheel_names:
				if wheel_name in self.pid_regression:
					kp = self.pid_regression[wheel_name]["kp"]
					ki = self.pid_regression[wheel_name]["ki"]
					kd = self.pid_regression[wheel_name]["kd"]
	
	@staticmethod
	def _linear_regression(x: List[float], y: List[float]) -> Tuple[float, float]:
		"""Calculate linear regression coefficients (y = mx + b)."""
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
	
	def set_navigator_rotation_gains(self, kp, ki, kd):
		"""Set navigator rotation PID gains"""
		self.navigator_rotation_gains["kp"] = kp
		self.navigator_rotation_gains["ki"] = ki
		self.navigator_rotation_gains["kd"] = kd
		print(f"[NAV] Navigator rotation gains updated: KP={kp:.6f}, KI={ki:.6f}, KD={kd:.6f}")
	
	def get_navigator_rotation_gains(self):
		"""Get navigator rotation PID gains"""
		return self.navigator_rotation_gains.copy()
	
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
	
	def get_alignment_gains(self):
		"""Get wheel alignment PID gains from calibration file
		
		Returns dict with "kp", "ki", "kd" keys.
		Returns empty dict if alignment gains haven't been calibrated yet
		(gains should be loaded from calibration file on RoboRIO during operation).
		"""
		gains = self.full_data.get("alignment_gains", {})
		if not gains:
			print("[WARNING] Alignment gains not found - should be loaded from RoboRIO calibration file")
		return gains

	# ------------------------------------------------------------------
	# Odometry + IMU dead-reckoning calibration (docs/plans/odometry_imu_calibration_plan.md)
	# ------------------------------------------------------------------

	@staticmethod
	def _default_rotation_calibration() -> Dict:
		return {
			"imu_scale_factor": 1.0,
			"odometry_rotation_scale_factor": 1.0,
			"accuracy_target_deg": 2.0,
			"trial_history": []
		}

	@staticmethod
	def _default_translation_calibration() -> Dict:
		return {
			"wheel_scale_curve": {name: {"points": [], "m": 0.0, "b": 1.0} for name in WHEEL_NAMES},
			"reversal_backlash_cm": {"forward_axis": 0.0, "strafe_axis": 0.0, "diagonal_axis": 0.0},
			"accuracy_target_pct": 1.5,
			"trial_history": []
		}

	def get_rotation_calibration(self) -> Dict:
		"""Get the current rotation (N-spin) calibration state."""
		return self.rotation_calibration

	def add_rotation_trial(self, entry: Dict) -> None:
		"""Record one N-spin trial and multiplicatively update the current scale
		factors: new_factor = old_factor * (true_total / measured_total), where
		measured_total was itself measured under the old_factor. This converges
		correctly across repeated iterative trials (see
		docs/plans/odometry_imu_calibration_plan.md section 5) - a plain average
		of per-trial ratios would double-correct once a factor other than 1.0
		is already active.
		"""
		history = self.rotation_calibration.setdefault("trial_history", [])
		history.append(entry)

		imu_ratio = entry.get("imu_correction_ratio")
		if imu_ratio:
			self.rotation_calibration["imu_scale_factor"] = self.rotation_calibration.get("imu_scale_factor", 1.0) * imu_ratio

		odom_ratio = entry.get("odometry_correction_ratio")
		if odom_ratio:
			self.rotation_calibration["odometry_rotation_scale_factor"] = (
				self.rotation_calibration.get("odometry_rotation_scale_factor", 1.0) * odom_ratio
			)

		print(f"[ROT-CAL] Trial recorded: {entry}")
		print(f"[ROT-CAL] Current factors: imu={self.rotation_calibration['imu_scale_factor']:.5f} "
			f"odom={self.rotation_calibration['odometry_rotation_scale_factor']:.5f}")

	def set_rotation_accuracy_target(self, target_deg: float) -> None:
		self.rotation_calibration["accuracy_target_deg"] = target_deg

	def clear_rotation_calibration(self) -> None:
		self.rotation_calibration = self._default_rotation_calibration()

	def get_translation_calibration(self) -> Dict:
		"""Get the current translation (7-level) calibration state."""
		return self.translation_calibration

	def add_wheel_scale_point(self, wheel_name: str, speed_pct: float, scale_factor: float) -> None:
		"""Record one (speed, scale_factor) data point for a wheel and refit
		the linear speed -> scale_factor curve (same regression used for
		PID-gain-vs-battery-voltage).
		"""
		curve_data = self.translation_calibration.setdefault("wheel_scale_curve", {})
		wheel_curve = curve_data.setdefault(wheel_name, {"points": [], "m": 0.0, "b": 1.0})
		wheel_curve["points"].append({"speed_pct": speed_pct, "scale_factor": scale_factor})

		speeds = [p["speed_pct"] for p in wheel_curve["points"]]
		factors = [p["scale_factor"] for p in wheel_curve["points"]]
		m, b = self._linear_regression(speeds, factors)
		wheel_curve["m"] = m
		wheel_curve["b"] = b

	def get_wheel_scale_curve(self, wheel_name: str) -> Dict:
		curve_data = self.translation_calibration.get("wheel_scale_curve", {})
		return curve_data.get(wheel_name, {"points": [], "m": 0.0, "b": 1.0})

	def set_reversal_backlash(self, axis: str, backlash_cm: float) -> None:
		backlash = self.translation_calibration.setdefault("reversal_backlash_cm", {})
		backlash[axis] = backlash_cm

	def get_reversal_backlash(self) -> Dict:
		return self.translation_calibration.get("reversal_backlash_cm", {})

	def add_translation_trial(self, entry: Dict) -> None:
		history = self.translation_calibration.setdefault("trial_history", [])
		history.append(entry)
		print(f"[TRANS-CAL] Trial recorded: {entry}")

	def set_translation_accuracy_target(self, target_pct: float) -> None:
		self.translation_calibration["accuracy_target_pct"] = target_pct

	def clear_translation_calibration(self) -> None:
		self.translation_calibration = self._default_translation_calibration()

	def discard_odometry_calibration_changes(self) -> None:
		"""Reload the rotation/translation calibration sections from disk, discarding
		any in-memory changes made during an unsaved calibration session.
		"""
		on_disk = {}
		try:
			path = self.get_calibration_path()
			if os.path.exists(path):
				with open(path, "r") as f:
					on_disk = json.load(f)
		except Exception as e:
			print(f"[DISCARD] Error reloading calibration: {e}")
		self.rotation_calibration = on_disk.get("rotation_calibration", self._default_rotation_calibration())
		self.translation_calibration = on_disk.get("translation_calibration", self._default_translation_calibration())
