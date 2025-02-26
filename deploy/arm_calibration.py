import time

def calibrate_encoders(arm):
	"""
	Automatically calibrates the shoulder and wrist encoders by asking the user 
	to manually move the arm to specific positions (90° and 180°) and confirming input.
	"""
	print("\n[CALIBRATION] Move the SHOULDER to exactly 90° and press Enter.")
	input()  # Wait for user confirmation
	shoulder_90 = arm.shoulder_encoder.getPosition()

	print("[CALIBRATION] Move the SHOULDER to exactly 180° and press Enter.")
	input()
	shoulder_180 = arm.shoulder_encoder.getPosition()

	# Compute degrees per tick for the shoulder
	arm.shoulder_deg_per_tick = (180 - 90) / (shoulder_180 - shoulder_90)
	arm.shoulder_zero_offset = shoulder_90 - (90 / arm.shoulder_deg_per_tick)

	print(f"\n[CALIBRATION] SHOULDER Complete:\n"
		  f" - Encoder at 90°: {shoulder_90}\n"
		  f" - Encoder at 180°: {shoulder_180}\n"
		  f" - Degrees per Tick: {arm.shoulder_deg_per_tick:.4f}\n"
		  f" - Zero Offset: {arm.shoulder_zero_offset:.4f}")

	# Wrist calibration
	print("\n[CALIBRATION] Move the WRIST to exactly 90° and press Enter.")
	input()
	wrist_90 = arm.wrist_encoder.getPosition()

	print("[CALIBRATION] Move the WRIST to exactly 180° and press Enter.")
	input()
	wrist_180 = arm.wrist_encoder.getPosition()

	arm.wrist_deg_per_tick = (180 - 90) / (wrist_180 - wrist_90)
	arm.wrist_zero_offset = wrist_90 - (90 / arm.wrist_deg_per_tick)

	print(f"\n[CALIBRATION] WRIST Complete:\n"
		  f" - Encoder at 90°: {wrist_90}\n"
		  f" - Encoder at 180°: {wrist_180}\n"
		  f" - Degrees per Tick: {arm.wrist_deg_per_tick:.4f}\n"
		  f" - Zero Offset: {arm.wrist_zero_offset:.4f}")

	# Brief delay to prevent unintended re-triggering
	time.sleep(1)
