// Apply min/max limits to sliders
document.getElementById("elevatorControl").min = LIMITS.ELEVATOR_MIN;
document.getElementById("elevatorControl").max = LIMITS.ELEVATOR_MAX;

document.getElementById("armControl").min = LIMITS.ARM_MIN;
document.getElementById("armControl").max = LIMITS.ARM_MAX;

document.getElementById("wristControl").min = LIMITS.WRIST_MIN;
document.getElementById("wristControl").max = LIMITS.WRIST_MAX;

const modeToggle = document.getElementById("modeToggle");
const positionInput = document.getElementById("positionName");
const savePositionButton = document.getElementById("savePosition");

// Slider controls
document.getElementById("elevatorControl").addEventListener("input", (e) => {
	elevatorHeight = Math.max(LIMITS.ELEVATOR_MIN, Math.min(LIMITS.ELEVATOR_MAX, parseInt(e.target.value)));
	draw();
});

document.getElementById("armControl").addEventListener("input", (e) => {
	armAngle = Math.max(LIMITS.ARM_MIN, Math.min(LIMITS.ARM_MAX, parseInt(e.target.value)));
	draw();
});

document.getElementById("wristControl").addEventListener("input", (e) => {
	wristAngle = Math.max(LIMITS.WRIST_MIN, Math.min(LIMITS.WRIST_MAX, parseInt(e.target.value)));
	draw();
});

// Grabber Button Controls
document.getElementById("grabberLoad").addEventListener("click", () => {
	grabberRotationSpeed = -2; // Rotate counterclockwise
});

document.getElementById("grabberUnload").addEventListener("click", () => {
	grabberRotationSpeed = 2; // Rotate clockwise
});

document.getElementById("grabberStop").addEventListener("click", () => {
	grabberRotationSpeed = 0; // Stop rotation
});

// Animation loop for grabber rotation
if (typeof lastTimestamp === "undefined") {
	let lastTimestamp = 0; 
}
function updateGrabberRotation(timestamp) {
	const deltaTime = timestamp - lastTimestamp;
	lastTimestamp = timestamp;

	if (grabberRotationSpeed !== 0) {
		grabberAngle = (grabberAngle + grabberRotationSpeed * (deltaTime / 16.67)) % 360; // Rotate smoothly
		draw();
	}

	requestAnimationFrame(updateGrabberRotation);
}

function updateModeUI() {
	console.log(`Updating UI for mode: ${window.mode}`); // Debugging

	const isDrawingMode = window.mode === "drawing";

	// Show/hide position input and save button
	positionInput.style.display = isDrawingMode ? "inline-block" : "none";
	savePositionButton.style.display = isDrawingMode ? "inline-block" : "none";

	// Show/hide delete buttons in saved positions
	const deleteButtons = document.querySelectorAll("#savedPositions button.delete");
	deleteButtons.forEach(button => {
		button.style.display = isDrawingMode ? "inline-block" : "none";
	});
}


modeToggle.addEventListener("click", () => {
	// Ensure mode is updated globally
	window.mode = window.mode === "drawing" ? "driving" : "drawing";

	modeToggle.textContent = window.mode === "drawing" ? "Switch to Driving Mode" : "Switch to Drawing Mode";

	console.log(`Mode changed to: ${window.mode}`); // Debugging

	updateModeUI(); // This ensures UI updates when the mode is toggled
	draw();
});


// Ensure UI is updated when the page loads
requestAnimationFrame(updateGrabberRotation);

window.onload = function () {
	if (typeof loadSavedPositions === "function") {
		loadSavedPositions(); // Ensure function is defined before calling
	}
	updateModeUI();
};
