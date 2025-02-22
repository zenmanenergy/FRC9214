function animateMove(targetPos) {
	const steps = 20;
	let step = 0;

	const start = {
		elevatorHeight,
		armAngle,
		wristAngle,
		grabberAngle,
	};

	function interpolate(startValue, endValue) {
		return startValue + (endValue - startValue) * (step / steps);
	}

	function stepMove() {
		if (step >= steps) {
			elevatorHeight = targetPos.elevatorHeight;
			armAngle = Math.max(ARM_MIN_ANGLE, Math.min(ARM_MAX_ANGLE, targetPos.armAngle)); // Limit within bounds
			wristAngle = Math.max(WRIST_MIN_ANGLE, Math.min(WRIST_MAX_ANGLE, targetPos.wristAngle)); // Limit within bounds
			grabberAngle = targetPos.grabberAngle;
			draw();
			return;
		}

		elevatorHeight = interpolate(start.elevatorHeight, targetPos.elevatorHeight);
		armAngle = Math.max(ARM_MIN_ANGLE, Math.min(ARM_MAX_ANGLE, interpolate(start.armAngle, targetPos.armAngle)));
		wristAngle = Math.max(WRIST_MIN_ANGLE, Math.min(WRIST_MAX_ANGLE, interpolate(start.wristAngle, targetPos.wristAngle)));
		grabberAngle = interpolate(start.grabberAngle, targetPos.grabberAngle);

		draw();
		step++;
		setTimeout(stepMove, 50);
	}

	stepMove();
}
