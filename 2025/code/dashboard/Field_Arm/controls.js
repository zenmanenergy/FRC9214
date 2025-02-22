const LIMITS = {
	ELEVATOR_MIN: 0,
	ELEVATOR_MAX: 100,
	ARM_MIN: -60,
	ARM_MAX: 200,
	WRIST_MIN: -120,
	WRIST_MAX: 120
};

// Track grabber rotation speed
let grabberRotationSpeed = 0; // Default is no movement
let lastTimestamp = 0;

// Apply min/max limits to sliders
document.getElementById("elevatorControl").min = LIMITS.ELEVATOR_MIN;
document.getElementById("elevatorControl").max = LIMITS.ELEVATOR_MAX;

document.getElementById("armControl").min = LIMITS.ARM_MIN;
document.getElementById("armControl").max = LIMITS.ARM_MAX;

document.getElementById("wristControl").min = LIMITS.WRIST_MIN;
document.getElementById("wristControl").max = LIMITS.WRIST_MAX;

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
function updateGrabberRotation(timestamp) {
	const deltaTime = timestamp - lastTimestamp;
	lastTimestamp = timestamp;

	if (grabberRotationSpeed !== 0) {
		grabberAngle = (grabberAngle + grabberRotationSpeed * (deltaTime / 16.67)) % 360; // Rotate smoothly
		draw();
	}

	requestAnimationFrame(updateGrabberRotation);
}

requestAnimationFrame(updateGrabberRotation);
