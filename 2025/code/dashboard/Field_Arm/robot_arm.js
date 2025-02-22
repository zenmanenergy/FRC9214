let elevatorHeight = 50;
let armAngle = 0;
let wristAngle = 0;
let grabberAngle = 0;

function drawRobotArm() {
	// DO NOT call drawField() here to prevent infinite recursion

	const baseX = 750;
	const baseY = 400;
	const elevatorHeightPixels = elevatorHeight * 2;

	// Base: Upside-down T
	ctx.fillStyle = "black";
	ctx.fillRect(baseX - 40, baseY - 20, 80, 20);
	ctx.fillRect(baseX - 10, baseY - 80, 20, 80);

	// Elevator
	ctx.fillStyle = "gray";
	ctx.fillRect(baseX - 5, baseY - 80 - elevatorHeightPixels, 10, elevatorHeightPixels);

	// Arm
	const armLength = 120;
	const armX = baseX;
	const armY = baseY - 80 - elevatorHeightPixels;
	const armEndX = armX + armLength * Math.cos((-armAngle * Math.PI) / 180);
	const armEndY = armY + armLength * Math.sin((-armAngle * Math.PI) / 180);
	ctx.strokeStyle = "blue";
	ctx.lineWidth = 8;
	ctx.beginPath();
	ctx.moveTo(armX, armY);
	ctx.lineTo(armEndX, armEndY);
	ctx.stroke();

	// Wrist
	const wristLength = 50;
	const wristEndX = armEndX + wristLength * Math.cos((-(armAngle + wristAngle) * Math.PI) / 180);
	const wristEndY = armEndY + wristLength * Math.sin((-(armAngle + wristAngle) * Math.PI) / 180);
	ctx.strokeStyle = "red";
	ctx.lineWidth = 6;
	ctx.beginPath();
	ctx.moveTo(armEndX, armEndY);
	ctx.lineTo(wristEndX, wristEndY);
	ctx.stroke();

	// Grabber
	const grabberX = wristEndX;
	const grabberY = wristEndY;
	const grabberRadius = 20;

	ctx.strokeStyle = "black";
	ctx.lineWidth = 3;
	ctx.beginPath();
	ctx.arc(grabberX, grabberY, grabberRadius, 0, Math.PI * 2); // Draw outer circle
	ctx.stroke();

	// Draw rotating spokes
	const spokeLength = 15;
	for (let i = 0; i < 4; i++) {
		const angle = ((i * 90 + grabberAngle) * Math.PI) / 180; // Apply grabber rotation
		const spokeX = grabberX + spokeLength * Math.cos(angle);
		const spokeY = grabberY + spokeLength * Math.sin(angle);
		ctx.beginPath();
		ctx.moveTo(grabberX, grabberY);
		ctx.lineTo(spokeX, spokeY);
		ctx.stroke();
	}

	// Draw the Tusk (backwards 7 shape wrapping around 1/4 of grabber)
	const tuskStartAngle = (-armAngle - wristAngle) * (Math.PI / 180); // Align with wrist rotation

	// Attach the tusk to the **left** side of the grabber at 0 degrees, shifted 5px left
	const tuskBaseX = grabberX + (grabberRadius * Math.cos(tuskStartAngle + Math.PI)) - 5;
	const tuskBaseY = grabberY + grabberRadius * Math.sin(tuskStartAngle + Math.PI);

	// End of the vertical part of the "7"
	const tuskEndX = tuskBaseX + 28 * Math.cos(tuskStartAngle - Math.PI / 2); // Vertical segment
	const tuskEndY = tuskBaseY + 28 * Math.sin(tuskStartAngle - Math.PI / 2);

	// Hook part of the "7" (90-degree turn instead of 45-degree)
	const tuskHookX = tuskEndX + 60 * Math.cos(tuskStartAngle); // 90-degree turn
	const tuskHookY = tuskEndY + 60 * Math.sin(tuskStartAngle);

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
