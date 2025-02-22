let elevatorHeight = 50;
let armAngle = 0;
let wristAngle = 0;
let grabberAngle = 0;



function drawRobotArm() {
	// DO NOT call drawField() here to prevent infinite recursion

	const baseX = 750;
	const baseY = 400;
	const elevatorHeightPixels = Math.max(ELEVATOR_MIN_HEIGHT, Math.min(ELEVATOR_MAX_HEIGHT, elevatorHeight * SCALE));

	// Wheels (draw first so they appear behind the base)
	const wheelY = baseY; // Center of the wheels aligns with the middle of BASE_HEIGHT

	ctx.fillStyle = "black";
	ctx.beginPath();
	ctx.arc(baseX - WHEEL_OFFSET, wheelY, WHEEL_RADIUS, 0, Math.PI * 2); // Left wheel
	ctx.fill();
	ctx.beginPath();
	ctx.arc(baseX + WHEEL_OFFSET, wheelY, WHEEL_RADIUS, 0, Math.PI * 2); // Right wheel
	ctx.fill();
	ctx.beginPath();
	ctx.arc(baseX + CENTER_WHEEL_OFFSET, wheelY, WHEEL_RADIUS, 0, Math.PI * 2); // Center wheel
	ctx.fill();

	// Base: Upside-down T
	ctx.fillStyle = "black";
	ctx.fillRect(baseX - BASE_VERTICAL_SECTION_WIDTH / 2, baseY - BASE_VERTICAL_SECTION_HEIGHT, BASE_VERTICAL_SECTION_WIDTH, BASE_VERTICAL_SECTION_HEIGHT);

	// Set base color based on team
	ctx.fillStyle = selectedTeam === "red" ? "red" : "blue";
	ctx.fillRect(baseX - BASE_WIDTH / 2, baseY - BASE_HEIGHT / 2, BASE_WIDTH, BASE_HEIGHT);

	// Add "9214" on top of the base rectangle
	ctx.fillStyle = "white";
	ctx.font = `${8 * SCALE}px Arial`; // Scale text size dynamically
	ctx.textAlign = "center";
	ctx.textBaseline = "middle";
	ctx.fillText("9214", baseX, baseY); // Center the text on the base

	// Elevator
	ctx.fillStyle = "gray";
	ctx.fillRect(baseX - 5, baseY - BASE_VERTICAL_SECTION_HEIGHT - elevatorHeightPixels, 10, elevatorHeightPixels);

	// Arm
	const armX = baseX;
	const armY = baseY - BASE_VERTICAL_SECTION_HEIGHT - elevatorHeightPixels;
	const armEndX = armX + ARM_LENGTH * Math.cos((-armAngle * Math.PI) / 180);
	const armEndY = armY + ARM_LENGTH * Math.sin((-armAngle * Math.PI) / 180);
	ctx.strokeStyle = "blue";
	ctx.lineWidth = 8;
	ctx.beginPath();
	ctx.moveTo(armX, armY);
	ctx.lineTo(armEndX, armEndY);
	ctx.stroke();

	// Wrist
	const wristEndX = armEndX + WRIST_LENGTH * Math.cos((-(armAngle + wristAngle) * Math.PI) / 180);
	const wristEndY = armEndY + WRIST_LENGTH * Math.sin((-(armAngle + wristAngle) * Math.PI) / 180);
	ctx.strokeStyle = "red";
	ctx.lineWidth = 6;
	ctx.beginPath();
	ctx.moveTo(armEndX, armEndY);
	ctx.lineTo(wristEndX, wristEndY);
	ctx.stroke();

	// Grabber
	const grabberX = wristEndX;
	const grabberY = wristEndY;
	ctx.strokeStyle = "black";
	ctx.lineWidth = 3;
	ctx.beginPath();
	ctx.arc(grabberX, grabberY, GRABBER_RADIUS, 0, Math.PI * 2);
	ctx.stroke();

	// Draw rotating spokes
	for (let i = 0; i < 4; i++) {
		const angle = ((i * 90 + grabberAngle) * Math.PI) / 180;
		const spokeX = grabberX + spokeLength * Math.cos(angle);
		const spokeY = grabberY + spokeLength * Math.sin(angle);
		ctx.beginPath();
		ctx.moveTo(grabberX, grabberY);
		ctx.lineTo(spokeX, spokeY);
		ctx.stroke();
	}

	// Draw the Tusk (backwards 7 shape wrapping around 1/4 of grabber)
	const tuskStartAngle = (-armAngle - wristAngle) * (Math.PI / 180);

	// Attach the tusk to the **left** side of the grabber at 0 degrees, shifted 5px left
	const tuskBaseX = grabberX + (GRABBER_RADIUS * Math.cos(tuskStartAngle + Math.PI)) - 5;
	const tuskBaseY = grabberY + GRABBER_RADIUS * Math.sin(tuskStartAngle + Math.PI);

	// End of the vertical part of the "7"
	const tuskEndX = tuskBaseX + TUSK_VERTICAL * Math.cos(tuskStartAngle - Math.PI / 2);
	const tuskEndY = tuskBaseY + TUSK_VERTICAL * Math.sin(tuskStartAngle - Math.PI / 2);

	// Hook part of the "7" (90-degree turn instead of 45-degree)
	const tuskHookX = tuskEndX + TUSK_HOOK * Math.cos(tuskStartAngle);
	const tuskHookY = tuskEndY + TUSK_HOOK * Math.sin(tuskStartAngle);

	ctx.strokeStyle = "black";
	ctx.lineWidth = 4;
	ctx.beginPath();
	ctx.moveTo(tuskBaseX, tuskBaseY);
	ctx.lineTo(tuskEndX, tuskEndY);
	ctx.lineTo(tuskHookX, tuskHookY);
	ctx.stroke();
}

// Ensure `drawRobotArm()` can be called externally
window.drawRobotArm = drawRobotArm;


// Ensure `drawRobotArm()` can be called externally
window.drawRobotArm = drawRobotArm;
