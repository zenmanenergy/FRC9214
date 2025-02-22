const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");
const teamSelect = document.getElementById("teamSelect");

let image = new Image();
let team = "blue"; // Default team

function loadImage() {
	image.src = "REEFSCAPE2025.png";
	image.onload = () => draw();
}

teamSelect.addEventListener("change", () => {
	team = teamSelect.value;
	draw();
});

function draw() {
	ctx.clearRect(0, 0, canvas.width, canvas.height);

	// Draw Field Map (Left 500px)
	let clipX = team === "blue" ? 0 : image.width / 2;
	ctx.drawImage(image, clipX, 0, image.width / 2, image.height, 0, 0, 500, 500);

	// Draw Robot Arm (Right 500px)
	drawRobotArm();
}

loadImage();
