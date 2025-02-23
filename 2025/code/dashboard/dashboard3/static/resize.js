function resizeToFitDriverStation() {
	// Estimated FRC Driver Station height (adjust if needed)
	const driverStationHeight = 300; 

	// Get screen dimensions
	const screenWidth = window.screen.width;
	const screenHeight = window.screen.height;

	// Set window size to fill the remaining space
	const newWidth = screenWidth; // Full width
	const newHeight = screenHeight - driverStationHeight; // Leave space for DS

	// Move and resize window
	window.resizeTo(newWidth, newHeight);
	window.moveTo(0, driverStationHeight);

	console.log(`🔧 Resized to: ${newWidth}x${newHeight}`);
}

// Resize window when loaded
window.onload = resizeToFitDriverStation;

// Optional: Recalculate size when manually resized
window.addEventListener("resize", resizeToFitDriverStation);
