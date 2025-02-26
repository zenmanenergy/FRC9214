const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

let fieldImage = new Image();

// Load the field image
fieldImage.src = "/static/REEFSCAPE2025.png";
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



// Export function for use in other files
window.drawField = drawField;
