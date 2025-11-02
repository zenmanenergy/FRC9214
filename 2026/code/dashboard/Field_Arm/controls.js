const socket = io.connect("http://" + location.hostname + ":5805", {transports: ['websocket']});

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

// WebSocket Connected ✅
socket.on("connect", function() {
	console.log("✅ WebSocket Connected to NetworkTables!");
});

// Listen for real robot data and update UI
socket.on("update_data", function(data) {
	console.log("📡 Received data:", data);
	
	// Update real-time values from the robot
	document.getElementById("x_position").innerText = data.x_position.toFixed(2) ?? "N/A";
	document.getElementById("y_position").innerText = data.y_position.toFixed(2) ?? "N/A";
	document.getElementById("elevator_value").innerText = data.elevator.toFixed(2) ?? "N/A";
	document.getElementById("arm_angle").innerText = data.arm_angle.toFixed(2) ?? "N/A";
	document.getElementById("wrist_angle").innerText = data.wrist_angle.toFixed(2) ?? "N/A";
	document.getElementById("grabber_angle").innerText = data.grabber_angle.toFixed(2) ?? "N/A";
});

// Function to send commands to the robot
function sendCommand(command) {
	console.log("🛠 Sending Command:", command);
	socket.emit("send_command", command);
}

// Slider controls - Send commands when changed
document.getElementById("elevatorControl").addEventListener("input", (e) => {
	const value = parseFloat(e.target.value);
	sendCommand({ elevator: value });
	elevatorHeight = value;
	draw();
});

document.getElementById("armControl").addEventListener("input", (e) => {
	const value = parseFloat(e.target.value);
	sendCommand({ arm_angle: value });
	armAngle = value;
	draw();
});

document.getElementById("wristControl").addEventListener("input", (e) => {
	const value = parseFloat(e.target.value);
	sendCommand({ wrist_angle: value });
	wristAngle = value;
	draw();
});

// Grabber Button Controls
document.getElementById("grabberLoad").addEventListener("click", () => {
	sendCommand({ grabber_angle: -30 }); // Rotate grabber counterclockwise
});

document.getElementById("grabberUnload").addEventListener("click", () => {
	sendCommand({ grabber_angle: 30 }); // Rotate grabber clockwise
});

document.getElementById("grabberStop").addEventListener("click", () => {
	sendCommand({ grabber_angle: 0 }); // Stop grabber
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

// Toggle between Drawing & Driving Mode
modeToggle.addEventListener("click", () => {
	window.mode = window.mode === "drawing" ? "driving" : "drawing";

	modeToggle.textContent = window.mode === "drawing" ? "Switch to Driving Mode" : "Switch to Drawing Mode";
	console.log(`Mode changed to: ${window.mode}`); // Debugging

	updateModeUI();
	draw();
});

// Team Selection Handler
document.getElementById("teamSelect").addEventListener("change", (event) => {
	selectedTeam = event.target.value; // Update team selection
	drawField(); // Redraw the field with the correct clipping
	draw(); // Redraw the robot with the correct color
});

// Ensure the Save Position button works
if (savePositionButton) {
	savePositionButton.addEventListener("click", () => {
		console.log("Save Position button clicked"); // Debugging
		savePosition(); // Call function from training.js
	});
}

// Ensure UI is updated when the page loads
requestAnimationFrame(updateGrabberRotation);

window.onload = function () {
	if (typeof loadSavedPositions === "function") {
		loadSavedPositions(); // Ensure function is defined before calling
	}
	updateModeUI();
};
