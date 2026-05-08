import wpilib
from wpilib import SmartDashboard, RobotController
import json
import traceback


class SwerveTuner:
	
	def __init__(self, wheels, pid_controllers, calibration):
		self.wheels = wheels
		self.pid_controllers = pid_controllers
		self.calibration = calibration
		
		self.autotuning = False
		self.gains = None
	
	def start(self):
		self.autotuning = True
		self.gains = {
			"wheels": ["front_left", "front_right", "rear_left", "rear_right"],
			"current_index": 0,
			"results": [],
			"wheel_start_time": wpilib.Timer.getFPGATimestamp(),
			"current": {
				"angles": [0, 90, 180, 270],
				"angle_index": 0,
				"kc_list": [],
				"tc_list": [],
				"sign_changes": 0,
				"last_error": 0,
				"kp": 0.009,
				"angle_start_time": wpilib.Timer.getFPGATimestamp(),
				"step": 0
			}
		}
	
	def is_active(self):
		return self.autotuning
	
	def update(self):
		if not self.autotuning:
			return
		
		gains = self.gains
		current_idx = gains["current_index"]
		wheel_name = gains["wheels"][current_idx]
		wheel = self.wheels[wheel_name]
		
		wheel_elapsed = wpilib.Timer.getFPGATimestamp() - gains["wheel_start_time"]
		
		if wheel_elapsed > 30:
			self._finalize_wheel()
			return
		
		current = gains["current"]
		angle_index = current["angle_index"]
		target_angle = current["angles"][angle_index]
		
		angle_elapsed = wpilib.Timer.getFPGATimestamp() - current["angle_start_time"]
		if angle_elapsed > 6:
			current["kc_list"].append(None)
			current["tc_list"].append(None)
			self._next_angle_or_finalize()
			return
		
		current_angle = wheel.get_angle()
		error = target_angle - current_angle
		if error > 180:
			error -= 360
		elif error < -180:
			error += 360
		
		if error * current["last_error"] < 0:
			current["sign_changes"] += 1
		current["last_error"] = error
		
		speed = max(-0.5, min(0.5, error * current["kp"]))
		wheel.set_turn_power(speed)
		
		step = int(angle_elapsed / 1.2)
		if step > current["step"]:
			current["step"] = step
			old_kp = current["kp"]
			current["kp"] *= 1.5
		
		if current["sign_changes"] >= 8 and len(current["kc_list"]) == angle_index:
			kc = current["kp"]
			current["kc_list"].append(kc)
			current["osc_start"] = wpilib.Timer.getFPGATimestamp()
			current["sign_changes"] = 0
		
		if len(current["kc_list"]) == angle_index + 1 and len(current["tc_list"]) == angle_index:
			if current["sign_changes"] >= 8:
				period_time = wpilib.Timer.getFPGATimestamp() - current["osc_start"]
				tc = period_time / 4.0
				current["tc_list"].append(tc)
				self._next_angle_or_finalize()
	
	def _next_angle_or_finalize(self):
		gains = self.gains
		current = gains["current"]
		current_idx = gains["current_index"]
		wheel_name = gains["wheels"][current_idx]
		
		current["angle_index"] += 1
		
		if current["angle_index"] < len(current["angles"]):
			current["angle_start_time"] = wpilib.Timer.getFPGATimestamp()
			current["kp"] = 0.009
			current["sign_changes"] = 0
			current["last_error"] = 0
			current["step"] = 0
		else:
			self._finalize_wheel()
	
	def _finalize_wheel(self):
		gains = self.gains
		current = gains["current"]
		current_idx = gains["current_index"]
		wheel_name = gains["wheels"][current_idx]
		self.wheels[wheel_name].set_turn_power(0)
		
		valid_kc = [kc for kc in current["kc_list"] if kc is not None]
		valid_tc = [tc for tc in current["tc_list"] if tc is not None]
		
		if valid_kc and valid_tc:
			avg_kc = sum(valid_kc) / len(valid_kc)
			avg_tc = sum(valid_tc) / len(valid_tc)
			
			kp = 0.6 * avg_kc
			ki = 0.8 * avg_kc / avg_tc if avg_tc > 0 else 0
			kd = 0.075 * avg_kc * avg_tc
		else:
			kp, ki, kd = 0.01, 0.002, 0.0001
		
		gains["results"].append({"wheel": wheel_name, "kp": kp, "ki": ki, "kd": kd})
		
		current_idx += 1
		if current_idx < len(gains["wheels"]):
			gains["current_index"] = current_idx
			gains["wheel_start_time"] = wpilib.Timer.getFPGATimestamp()
			gains["current"] = {
				"angles": [0, 90, 180, 270],
				"angle_index": 0,
				"kc_list": [],
				"tc_list": [],
				"sign_changes": 0,
				"last_error": 0,
				"kp": 0.009,
				"angle_start_time": wpilib.Timer.getFPGATimestamp(),
				"step": 0
			}
		else:
			self._compute_final_gains()
	
	def _compute_final_gains(self):
		gains = self.gains
		results = gains["results"]
		
		wheel_gains = {}
		for r in results:
			wheel_gains[r['wheel']] = {
				"kp": r['kp'],
				"ki": r['ki'],
				"kd": r['kd']
			}
		
		for wheel_name, pid in self.pid_controllers.items():
			if wheel_name in wheel_gains:
				g = wheel_gains[wheel_name]
				pid.set_gains(g['kp'], g['ki'], g['kd'])
		
		battery_voltage = RobotController.getBatteryVoltage()
		
		self.calibration.add_tuning_result(battery_voltage, wheel_gains)
		self.calibration.save_calibration()
		
		for wheel_name, gains in wheel_gains.items():
			SmartDashboard.putNumber(f"autotune_{wheel_name}_kp", gains['kp'])
			SmartDashboard.putNumber(f"autotune_{wheel_name}_ki", gains['ki'])
			SmartDashboard.putNumber(f"autotune_{wheel_name}_kd", gains['kd'])
		SmartDashboard.putNumber("autotune_battery_voltage", battery_voltage)
		SmartDashboard.putBoolean("autotune_complete", True)
		
		self.publish_tuning_history()
		
		self.autotuning = False
		self.gains = None
	
	def publish_tuning_history(self):
		try:
			history = self.calibration.pid_tuning_history
			
			history_json = json.dumps(history)
			
			SmartDashboard.putString("autotune_history_json", history_json)
			
			if hasattr(self.calibration, 'pid_regression') and self.calibration.pid_regression:
				regression_json = json.dumps(self.calibration.pid_regression)
				SmartDashboard.putString("autotune_regression_json", regression_json)
				
		except Exception:
			traceback.print_exc()
