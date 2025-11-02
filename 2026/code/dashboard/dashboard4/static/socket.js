const socket = io.connect("http://" + location.hostname + ":5805", {transports: ['websocket']});


// ✅ Show connected status in browser console
socket.on("connect", function () {
	console.log("✅ WebSocket Connected to Server!");
	socket.emit("client_connected"); // Notify server
});

// ✅ Show disconnected status in browser console
socket.on("disconnect", function () {
	console.log("❌ WebSocket Disconnected!");
});

// ✅ Show NetworkTables updates
// socket.on("update_data", function (data) {
// 	console.log("📡 Received data:", data);
// 	document.getElementById("x_position").innerText = (data.real_x_position ?? 0).toFixed(2);
// 	document.getElementById("y_position").innerText = (data.real_y_position ?? 0).toFixed(2);
// 	document.getElementById("elevator_value").innerText = (data.real_elevator ?? 0).toFixed(2);
// 	document.getElementById("arm_angle").innerText = (data.real_arm_angle ?? 0).toFixed(2);
// 	document.getElementById("wrist_angle").innerText = (data.real_wrist_angle ?? 0).toFixed(2);
// 	document.getElementById("grabber_angle").innerText = (data.real_grabber_angle ?? 0).toFixed(2);
// });

// Real robot state (from NetworkTables)
let realElevatorHeight = 50;
let realArmAngle = 0;
let realWristAngle = 0;
let realGrabberAngle = 0;

socket.on("update_data", function(data) {
	// console.log("📡 Received data:", data);

	// Ensure values are not undefined before calling .toFixed()
	document.getElementById("x_position").innerText = (data.real_x_position ?? 0).toFixed(2);
	document.getElementById("y_position").innerText = (data.real_y_position ?? 0).toFixed(2);
	document.getElementById("elevator_value").innerText = (data.real_elevator ?? 0).toFixed(2);
	document.getElementById("arm_angle").innerText = (data.real_arm_angle ?? 0).toFixed(2);
	document.getElementById("wrist_angle").innerText = (data.real_wrist_angle ?? 0).toFixed(2);
	document.getElementById("grabber_angle").innerText = (data.real_grabber_angle ?? 0).toFixed(2);

	// Update real robot values for visualization
	realElevatorHeight = data.real_elevator ?? 0;
	realArmAngle = data.real_arm_angle ?? 0;
	realWristAngle = data.real_wrist_angle ?? 0;
	realGrabberAngle = data.real_grabber_angle ?? 0;

	// Redraw arm with both real and commanded positions
	drawRobotArm();
});

// Function to send commands to the robot
function sendCommand(command) {
	console.log("🛠 Sending Command:", command);
	socket.emit("send_command", command);
}

// Attach commands to sliders
document.getElementById("elevatorControl").addEventListener("input", (e) => {
	sendCommand({ cmd_elevator: parseFloat(e.target.value) });
});
document.getElementById("armControl").addEventListener("input", (e) => {
	sendCommand({ cmd_arm_angle: parseFloat(e.target.value) });
});
document.getElementById("wristControl").addEventListener("input", (e) => {
	sendCommand({ cmd_wrist_angle: parseFloat(e.target.value) });
});

// Grabber button controls
document.getElementById("grabberLoad").addEventListener("click", () => {
	sendCommand({ cmd_grabber_angle: -30 });
});
document.getElementById("grabberUnload").addEventListener("click", () => {
	sendCommand({ cmd_grabber_angle: 30 });
});
document.getElementById("grabberStop").addEventListener("click", () => {
	sendCommand({ cmd_grabber_angle: 0 });
});
