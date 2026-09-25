"""Odometry + IMU dead-reckoning calibration routines.

Implements the two guided calibration routines described in
docs/plans/odometry_imu_calibration_plan.md:

  - RotationCalibrator: the "N-spin" test, calibrating the IMU and the
    wheel-odometry rotation math together from the same physical trial.
  - TranslationCalibrator: the 7-level forward/strafe/diagonal test suite,
    calibrating per-wheel distance-scale (as a function of speed) and
    reversal backlash.

Both are driven one trial at a time by CalibrationModeHandler, which bridges
NetworkTables flags (from the dashboard) to the methods below. Nothing is
persisted to the calibration file until the operator applies it - see
add_rotation_trial()/add_translation_trial() on EncoderCalibration.
"""

from typing import Dict, List, Optional, Tuple
import math
from datetime import datetime


def _shortest_delta(prev_deg: float, cur_deg: float) -> float:
	"""Shortest signed angular delta from prev_deg to cur_deg (degrees)."""
	delta = cur_deg - prev_deg
	if delta > 180:
		delta -= 360
	elif delta < -180:
		delta += 360
	return delta


class RotationCalibrator:
	"""The "N-spin" rotation calibration routine (see plan section 5)."""

	def __init__(self, drive, calibration) -> None:
		self.drive = drive
		self.calibration = calibration
		self.state = "idle"  # idle -> running -> awaiting_input -> awaiting_reset -> idle
		self.session_id: Optional[str] = None
		self.n = 1
		self.speed = 0.5
		self.target_total_deg = 0.0
		self._traveled_imu = 0.0
		self._traveled_odom = 0.0
		self._last_imu = 0.0
		self._last_odom = 0.0
		self.history: List[Dict] = []
		self.last_trial: Optional[Dict] = None

	def start_trial(self, n: int, speed_pct: float) -> None:
		self.n = max(1, int(n))
		self.speed = max(0.05, min(1.0, speed_pct / 100.0))
		self.target_total_deg = self.n * 360.0
		self._traveled_imu = 0.0
		self._traveled_odom = 0.0
		self._last_imu = self.drive.imu.get_heading()
		self._last_odom = self.drive.odometry.get_heading()
		self.state = "running"
		if self.session_id is None:
			self.session_id = datetime.now().isoformat()

	def update(self) -> None:
		"""Call every loop while state == 'running'."""
		if self.state != "running":
			return

		cur_imu = self.drive.imu.get_heading()
		self._traveled_imu += _shortest_delta(self._last_imu, cur_imu)
		self._last_imu = cur_imu

		cur_odom = self.drive.odometry.get_heading()
		self._traveled_odom += _shortest_delta(self._last_odom, cur_odom)
		self._last_odom = cur_odom
		self.drive.odometry.update()

		if abs(self._traveled_odom) >= self.target_total_deg - 2.0:
			self.drive.stop_all()
			self.state = "awaiting_input"
			return

		self.drive.rotate_in_place(self.speed)

	def cancel(self) -> None:
		self.drive.stop_all()
		self.state = "idle"

	def submit_residual(self, residual_deg: float) -> Dict:
		"""Operator eyeballed the mark; residual_deg is the small leftover angle."""
		true_total = self.n * 360.0 + residual_deg
		imu_ratio = true_total / self._traveled_imu if abs(self._traveled_imu) > 1e-6 else 1.0
		odom_ratio = true_total / self._traveled_odom if abs(self._traveled_odom) > 1e-6 else 1.0

		entry = {
			"timestamp": datetime.now().isoformat(),
			"session_id": self.session_id,
			"n": self.n,
			"speed_pct": round(self.speed * 100),
			"imu_reported_total": round(self._traveled_imu, 2),
			"odometry_reported_total": round(self._traveled_odom, 2),
			"residual_deg": residual_deg,
			"true_total": round(true_total, 2),
			"imu_correction_ratio": imu_ratio,
			"odometry_correction_ratio": odom_ratio,
		}
		self.calibration.add_rotation_trial(entry)

		rot_cal = self.calibration.get_rotation_calibration()
		self.drive.imu.set_scale_factor(rot_cal["imu_scale_factor"])
		self.drive.odometry.set_rotation_scale_factor(rot_cal["odometry_rotation_scale_factor"])

		entry["imu_scale_factor"] = rot_cal["imu_scale_factor"]
		entry["odometry_rotation_scale_factor"] = rot_cal["odometry_rotation_scale_factor"]
		self.history.append(entry)
		self.last_trial = entry
		self.state = "awaiting_reset"
		return entry

	def confirm_reset(self) -> None:
		self.state = "idle"

	def reset_session(self) -> None:
		"""Clear in-progress session state (called after apply/discard)."""
		self.drive.stop_all()
		self.state = "idle"
		self.session_id = None
		self.history = []
		self.last_trial = None

	def status(self) -> Dict:
		rot_cal = self.calibration.get_rotation_calibration()
		return {
			"state": self.state,
			"n": self.n,
			"speed_pct": round(self.speed * 100),
			"target_total_deg": self.target_total_deg,
			"traveled_imu": round(self._traveled_imu, 2),
			"traveled_odom": round(self._traveled_odom, 2),
			"imu_scale_factor": rot_cal["imu_scale_factor"],
			"odometry_rotation_scale_factor": rot_cal["odometry_rotation_scale_factor"],
			"accuracy_target_deg": rot_cal["accuracy_target_deg"],
			"last_trial": self.last_trial,
			"history": self.history[-20:],
		}


# ----------------------------------------------------------------------
# Translation calibration - 7-level test suite
# ----------------------------------------------------------------------

LEVELS: Dict[int, Dict] = {
	1: {"name": "Forward", "legs": [{"wheel_angle": 180, "sign": 1}]},
	2: {"name": "Right (strafe)", "legs": [{"wheel_angle": 90, "sign": 1}]},
	3: {"name": "Diagonal 45", "legs": [{"wheel_angle": 135, "sign": 1}]},
	4: {"name": "Forward -> Backward", "legs": [{"wheel_angle": 180, "sign": 1}, {"wheel_angle": 180, "sign": -1}]},
	5: {"name": "Right -> Left", "legs": [{"wheel_angle": 90, "sign": 1}, {"wheel_angle": 90, "sign": -1}]},
	6: {"name": "Diagonal 45 -> -45 (return)", "legs": [{"wheel_angle": 135, "sign": 1}, {"wheel_angle": 135, "sign": -1}]},
	7: {"name": "Forward -> Right (validation)", "legs": [{"wheel_angle": 180, "sign": 1}, {"wheel_angle": 90, "sign": 1}]},
}

AXIS_FOR_LEVEL = {1: "forward_axis", 2: "strafe_axis", 3: "diagonal_axis",
				  4: "forward_axis", 5: "strafe_axis", 6: "diagonal_axis"}

WHEEL_NAMES = ["front_left", "front_right", "rear_left", "rear_right"]

# Proportional gain for the per-wheel drift-feedback heuristic (plan section 6.4).
# This is a documented approximation, not a rigorous kinematic solve: it nudges
# per-wheel scale in the direction that reduces observed perpendicular drift,
# converging through repeated iteration rather than a one-shot least-squares fit.
_BIAS_GAIN = 2.0

DECEL_DISTANCE_CM = 20.0


def _leg_displacement_cm(wheel_angle: float, signed_dist_cm: float) -> Tuple[float, float]:
	"""Local-frame (dx, dy) for a pure-translation leg, matching SwerveOdometry's
	own per-wheel-vector + field-rotation math (heading=0, all wheels identical).
	"""
	a = math.radians(wheel_angle - 180.0)
	rx = math.cos(a) * signed_dist_cm
	ry = math.sin(a) * signed_dist_cm
	dx = ry
	dy = -rx
	return dx, dy


class TranslationCalibrator:
	"""The 7-level forward/strafe/diagonal calibration routine (plan section 6)."""

	def __init__(self, drive, calibration) -> None:
		self.drive = drive
		self.calibration = calibration
		self.state = "idle"
		self.session_id: Optional[str] = None
		self.level = 1
		self.x_meters = 1.0
		self.speed = 0.5
		self.leg_index = 0
		self._leg_start_distance = 0.0
		self.planned_path: List[Tuple[float, float]] = [(0.0, 0.0)]
		self.actual_path: List[Tuple[float, float]] = [(0.0, 0.0)]
		self.wheel_bias = {name: 0.0 for name in WHEEL_NAMES}
		self.history: List[Dict] = []
		self.last_trial: Optional[Dict] = None

	def _plan_path(self, level: int, x_m: float) -> List[Tuple[float, float]]:
		legs = LEVELS[level]["legs"]
		x_cm = x_m * 100.0
		pts = [(0.0, 0.0)]
		cx, cy = 0.0, 0.0
		for leg in legs:
			dx, dy = _leg_displacement_cm(leg["wheel_angle"], x_cm * leg["sign"])
			cx += dx
			cy += dy
			pts.append((cx, cy))
		return pts

	def start_trial(self, level: int, x_meters: float, speed_pct: float) -> None:
		self.level = level
		self.x_meters = max(0.1, x_meters)
		self.speed = max(0.05, min(1.0, speed_pct / 100.0))
		self.leg_index = 0
		self.drive.odometry.reset()
		self.planned_path = self._plan_path(level, self.x_meters)
		self.actual_path = [(0.0, 0.0)]
		self._begin_leg()
		self.state = "running"
		if self.session_id is None:
			self.session_id = datetime.now().isoformat()

	def _begin_leg(self) -> None:
		self._leg_start_distance = self.drive.odometry.get_distance_traveled()

	def update(self) -> None:
		if self.state != "running":
			return

		legs = LEVELS[self.level]["legs"]
		leg = legs[self.leg_index]
		target_cm = self.x_meters * 100.0

		self.drive.odometry.update()

		traveled = self.drive.odometry.get_distance_traveled() - self._leg_start_distance
		remaining = target_cm - traveled

		if remaining <= 0:
			self.drive.drive_straight(0.0, leg["wheel_angle"])
			self.actual_path.append(self.drive.odometry.get_position())
			self.leg_index += 1
			if self.leg_index >= len(legs):
				self.drive.stop_all()
				self.state = "awaiting_input"
			else:
				self._begin_leg()
			return

		ramped_speed = self.speed * min(1.0, remaining / DECEL_DISTANCE_CM)
		self.drive.drive_straight(ramped_speed * leg["sign"], leg["wheel_angle"])
		self.actual_path.append(self.drive.odometry.get_position())

	def cancel(self) -> None:
		self.drive.stop_all()
		self.state = "idle"

	def _apply_wheel_scale(self, wheel_scale: Dict[str, float]) -> None:
		for wheel_name, scale in wheel_scale.items():
			self.calibration.add_wheel_scale_point(wheel_name, round(self.speed * 100), scale)
			curve = self.calibration.get_wheel_scale_curve(wheel_name)
			self.drive.odometry.set_wheel_scale_curve(wheel_name, curve["m"], curve["b"])

	def submit_result(self, measured_distance_m: Optional[float] = None,
					   perpendicular_drift_cm: float = 0.0,
					   measured_x_cm: Optional[float] = None,
					   measured_y_cm: Optional[float] = None) -> Dict:
		"""Record operator tape-measure results for the just-completed level and
		live-apply the resulting correction (per plan section 6/7).
		"""
		level = self.level
		axis = AXIS_FOR_LEVEL.get(level)
		entry = {
			"timestamp": datetime.now().isoformat(),
			"session_id": self.session_id,
			"level": level,
			"level_name": LEVELS[level]["name"],
			"speed_pct": round(self.speed * 100),
			"x_meters": self.x_meters,
		}

		if level == 7:
			# Validation only - compare final measured local position to the
			# already-calibrated model's prediction. No corrections are fit here.
			pred_x, pred_y = self.planned_path[-1]
			mx = measured_x_cm if measured_x_cm is not None else pred_x
			my = measured_y_cm if measured_y_cm is not None else pred_y
			error_cm = math.hypot(mx - pred_x, my - pred_y)
			entry.update({
				"measured_x_cm": mx, "measured_y_cm": my,
				"predicted_x_cm": pred_x, "predicted_y_cm": pred_y,
				"error_cm": error_cm,
			})
		elif level in (1, 2, 3):
			measured_m = measured_distance_m if measured_distance_m is not None else self.x_meters
			current_avg_scale = self.drive.odometry.get_average_wheel_scale(self.speed)
			ratio = (measured_m / self.x_meters) if self.x_meters else 1.0
			new_avg_scale = current_avg_scale * ratio

			target_cm = self.x_meters * 100.0
			drift_norm = perpendicular_drift_cm / target_cm if target_cm else 0.0

			if level == 1:  # forward: drift indicates left/right bias
				self.wheel_bias["front_left"] += _BIAS_GAIN * drift_norm
				self.wheel_bias["rear_left"] += _BIAS_GAIN * drift_norm
				self.wheel_bias["front_right"] -= _BIAS_GAIN * drift_norm
				self.wheel_bias["rear_right"] -= _BIAS_GAIN * drift_norm
			elif level == 2:  # strafe: drift indicates front/rear bias
				self.wheel_bias["front_left"] += _BIAS_GAIN * drift_norm
				self.wheel_bias["front_right"] += _BIAS_GAIN * drift_norm
				self.wheel_bias["rear_left"] -= _BIAS_GAIN * drift_norm
				self.wheel_bias["rear_right"] -= _BIAS_GAIN * drift_norm
			else:  # diagonal cross-check: split half-weighted across both biases
				half = _BIAS_GAIN * drift_norm * 0.5
				self.wheel_bias["front_left"] += half
				self.wheel_bias["rear_left"] += half
				self.wheel_bias["front_right"] -= half
				self.wheel_bias["rear_right"] -= half

			wheel_scale = {name: new_avg_scale * (1.0 + self.wheel_bias[name]) for name in WHEEL_NAMES}
			self._apply_wheel_scale(wheel_scale)

			entry.update({
				"measured_distance_m": measured_m,
				"perpendicular_drift_cm": perpendicular_drift_cm,
				"new_avg_scale": new_avg_scale,
				"wheel_scale": wheel_scale,
				"wheel_bias": dict(self.wheel_bias),
			})
		else:  # levels 4-6: backlash detection
			measured_m = measured_distance_m if measured_distance_m is not None else self.x_meters
			shortfall_cm = (self.x_meters * 100.0) - (measured_m * 100.0)
			current_backlash = self.calibration.get_reversal_backlash().get(axis, 0.0)
			new_backlash = max(0.0, current_backlash + shortfall_cm)
			self.calibration.set_reversal_backlash(axis, new_backlash)
			self.drive.odometry.set_axis_backlash(axis, new_backlash)

			entry.update({
				"measured_distance_m": measured_m,
				"perpendicular_drift_cm": perpendicular_drift_cm,
				"axis": axis,
				"shortfall_cm": shortfall_cm,
				"backlash_cm": new_backlash,
			})

		self.calibration.add_translation_trial(entry)
		self.history.append(entry)
		self.last_trial = entry
		self.state = "awaiting_reset"
		return entry

	def confirm_reset(self) -> None:
		self.state = "idle"

	def reset_session(self) -> None:
		"""Clear in-progress session state (called after apply/discard)."""
		self.drive.stop_all()
		self.state = "idle"
		self.session_id = None
		self.history = []
		self.last_trial = None
		self.wheel_bias = {name: 0.0 for name in WHEEL_NAMES}

	def status(self) -> Dict:
		trans_cal = self.calibration.get_translation_calibration()
		wheel_scales = {
			name: self.drive.odometry.get_wheel_scale(name, self.speed)
			for name in WHEEL_NAMES
		}
		return {
			"state": self.state,
			"level": self.level,
			"level_name": LEVELS[self.level]["name"],
			"leg_index": self.leg_index,
			"leg_count": len(LEVELS[self.level]["legs"]),
			"x_meters": self.x_meters,
			"speed_pct": round(self.speed * 100),
			"planned_path_cm": self.planned_path,
			"actual_path_cm": self.actual_path[-200:],
			"wheel_scale_factors": wheel_scales,
			"backlash_cm": trans_cal.get("reversal_backlash_cm", {}),
			"accuracy_target_pct": trans_cal.get("accuracy_target_pct", 1.5),
			"last_trial": self.last_trial,
			"history": self.history[-20:],
		}
