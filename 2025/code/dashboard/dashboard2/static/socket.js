const socket = io.connect("http://" + location.hostname + ":5805", {transports: ['websocket']});

socket.on("connect", function() {
	console.log("✅ WebSocket Connected to NetworkTables!");
});

// Listen for updates from the robot and update UI
socket.on("update_data", function(data) {
	console.log("📡 Received data:", data);
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

// Attach commands to sliders
document.getElementById("elevatorControl").addEventListener("input", (e) => {
	sendCommand({ elevator: parseFloat(e.target.value) });
});
document.getElementById("armControl").addEventListener("input", (e) => {
	sendCommand({ arm_angle: parseFloat(e.target.value) });
});
document.getElementById("wristControl").addEventListener("input", (e) => {
	sendCommand({ wrist_angle: parseFloat(e.target.value) });
});

// Grabber button controls
document.getElementById("grabberLoad").addEventListener("click", () => {
	sendCommand({ grabber_angle: -30 });
});
document.getElementById("grabberUnload").addEventListener("click", () => {
	sendCommand({ grabber_angle: 30 });
});
document.getElementById("grabberStop").addEventListener("click", () => {
	sendCommand({ grabber_angle: 0 });
});
