import time

def average_measurements(measurements):
	"""Compute the average of a list of values."""
	return sum(measurements) / len(measurements)

def calibrate_position(arm, joint_name, encoder, target_degree):
	"""Prompt the user to move the arm to a specific degree position multiple times for better accuracy."""
	measurements = []
	for i in range(3):  # Repeat measurement 3 times
		print(f"\n[CALIBRATION] Move the {joint_name} to exactly {target_degree}° and press Enter. (Attempt {i+1}/3)")
		input()
		measurements.append(encoder.getPosition())

	average_value = average_measurements(measurements)
	print(f"[CALIBRATION] {joint_name} {target_degree}° Average Encoder Value: {average_value:.4f}")
	return average_value

def calibrate_encoders(arm):
	"""
	Automatically calibrates the shoulder and wrist encoders by asking the user 
	to manually move the arm to specific positions (90° and 180°) and averaging multiple readings.
	"""
	print("\n[CALIBRATION] Starting calibration...")

	# Shoulder Calibration
	shoulder_90_avg = calibrate_position(arm, "SHOULDER", arm.shoulder_encoder, 90)
	shoulder_180_avg = calibrate_position(arm, "SHOULDER", arm.shoulder_encoder, 180)

	# Compute degrees per tick for the shoulder
	arm.shoulder_deg_per_tick = (180 - 90) / (shoulder_180_avg - shoulder_90_avg)
	arm.shoulder_zero_offset = shoulder_90_avg - (90 / arm.shoulder_deg_per_tick)

	print(f"\n[CALIBRATION] SHOULDER Complete:\n"
		  f" - Encoder at 90°: {shoulder_90_avg:.4f}\n"
		  f" - Encoder at 180°: {shoulder_180_avg:.4f}\n"
		  f" - Degrees per Tick: {arm.shoulder_deg_per_tick:.4f}\n"
		  f" - Zero Offset: {arm.shoulder_zero_offset:.4f}")

	# Wrist Calibration
	wrist_90_avg = calibrate_position(arm, "WRIST", arm.wrist_encoder, 90)
	wrist_180_avg = calibrate_position(arm, "WRIST", arm.wrist_encoder, 180)

	arm.wrist_deg_per_tick = (180 - 90) / (wrist_180_avg - wrist_90_avg)
	arm.wrist_zero_offset = wrist_90_avg - (90 / arm.wrist_deg_per_tick)

	print(f"\n[CALIBRATION] WRIST Complete:\n"
		  f" - Encoder at 90°: {wrist_90_avg:.4f}\n"
		  f" - Encoder at 180°: {wrist_180_avg:.4f}\n"
		  f" - Degrees per Tick: {arm.wrist_deg_per_tick:.4f}\n"
		  f" - Zero Offset: {arm.wrist_zero_offset:.4f}")

	# Brief delay to prevent unintended re-triggering
	time.sleep(1)
