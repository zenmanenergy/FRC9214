const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

let fieldImage = new Image();
let selectedTeam = "blue"; // Default to Blue team

// Load the field image
fieldImage.src = "REEFSCAPE2025.png";
fieldImage.onload = () => {
	drawField();
};
function drawField() {
	ctx.clearRect(0, 0, canvas.width, canvas.height);

	// Clip to half the image
	const sourceWidth = fieldImage.width / 2;
	const sourceHeight = fieldImage.height;
	const sourceX = selectedTeam === "blue" ? 0 : sourceWidth;
	const sourceY = 0;

	// Keep original proportions
	const destWidth = canvas.width / 2;
	const destHeight = canvas.height;
	ctx.drawImage(fieldImage, sourceX, sourceY, sourceWidth, sourceHeight, 0, 0, destWidth, destHeight);

	// Draw the robot arm AFTER the field image so it remains visible
	window.drawRobotArm();
}



// Event listener for team selection dropdown
document.getElementById("teamSelect").addEventListener("change", (event) => {
	selectedTeam = event.target.value; // Update team selection
	drawField(); // Redraw the field with the correct clipping
});

// Export function for use in other files
window.drawField = drawField;
