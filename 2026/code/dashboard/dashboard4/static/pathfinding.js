

let robotImage = new Image();
let dots = [];
let connections = [];
let selectedDot = null;
let robot = null;
let path = [];
let robotAngle = 0;



// Robot image
robotImage.src = "https://tse1.mm.bing.net/th?id=OIP.k23XfFRFUpnXXNGcSlz-EgHaFj"; // Blue tank with turret

// Draw the field, robot, and path
// Ensure the field is drawn before pathfinding elements
function draw() {
	window.drawField(); // Calls drawField() from field.js

	// Draw connections
	connections.forEach(({ start, end }, index) => {
		ctx.beginPath();
		ctx.moveTo(dots[start].x, dots[start].y);
		ctx.lineTo(dots[end].x, dots[end].y);
		ctx.strokeStyle = path.includes(index) ? "green" : "black";
		ctx.stroke();
	});

	// Draw dots
	dots.forEach((dot, index) => {
		ctx.beginPath();
		ctx.arc(dot.x, dot.y, 5, 0, Math.PI * 2);
		console.log("x: " + (dot.x / 1.475))
		console.log("y: " + Math.abs(500-dot.y))
		ctx.fillStyle = index === selectedDot ? "red" : "blue";
		ctx.fill();
	});

	// Draw the robot
	if (robot !== null) {
		ctx.save();
		ctx.translate(dots[robot].x, dots[robot].y);
		ctx.rotate(robotAngle);
		ctx.drawImage(robotImage, -12, -12, 25, 25);
		ctx.restore();
	}

	saveData();
}

document.addEventListener("keydown", (event) => {
	if (event.key === "Delete" || event.key === "Backspace") {
		if (selectedDot !== null) {
			deleteDot(selectedDot);
			selectedDot = null; // Clear selection after deletion
			draw();
		}
	}
});

function deleteDot(dotIndex) {
	// Remove the dot from the array
	dots.splice(dotIndex, 1);

	// Remove all connections that include the deleted dot
	connections = connections.filter(conn => conn.start !== dotIndex && conn.end !== dotIndex);

	// Adjust connection indices since dots have shifted
	connections.forEach(conn => {
		if (conn.start > dotIndex) conn.start--;
		if (conn.end > dotIndex) conn.end--;
	});

	// Save updated data
	saveData();
}

// Find a dot that was clicked
function getClickedDot(x, y) {
	return dots.findIndex(dot => Math.hypot(dot.x - x, dot.y - y) < 5);
}

// Pathfinding algorithm (Breadth-First Search)
function findPath(start, end) {
	let queue = [[start]];
	let visited = new Set();

	while (queue.length > 0) {
		let path = queue.shift();
		let node = path[path.length - 1];

		if (node === end) return path;

		if (!visited.has(node)) {
			visited.add(node);
			let neighbors = connections
				.filter(conn => conn.start === node || conn.end === node)
				.map(conn => (conn.start === node ? conn.end : conn.start));

			neighbors.forEach(n => queue.push([...path, n]));
		}
	}
	return [];
}

// Animate robot movement along a path
function animateRobot(path) {
	let i = 0;
	function move() {
		if (i < path.length) {
			let nextDot = path[i];

			if (robot !== null) {
				let dx = dots[nextDot].x - dots[robot].x;
				let dy = dots[nextDot].y - dots[robot].y;
				robotAngle = Math.atan2(dy, dx);
			}

			robot = nextDot;
			draw();
			i++;
			setTimeout(move, 500);
		}
	}
	move();
}

// Handle canvas clicks
canvas.addEventListener("click", (e) => {
	const rect = canvas.getBoundingClientRect();
	const x = e.clientX - rect.left;
	const y = e.clientY - rect.top;
	const clickedIndex = getClickedDot(x, y);

	if (mode === "drawing") {
		if (clickedIndex !== -1) {
			if (selectedDot !== null && selectedDot !== clickedIndex) {
				connections.push({ start: selectedDot, end: clickedIndex });
				selectedDot = null;
			} else {
				selectedDot = clickedIndex;
			}
		} else {
			dots.push({ x, y });
		}
	} else if (mode === "driving" && clickedIndex !== -1) {
		if (robot === null) {
			robot = clickedIndex;
		} else {
			let pathIndices = findPath(robot, clickedIndex);
			if (pathIndices.length > 0) {
				path = pathIndices;
				animateRobot(path);
			}
		}
	}
	draw();
});

// Toggle between Drawing and Driving modes
document.getElementById("modeToggle").addEventListener("click", () => {
	mode = mode === "drawing" ? "driving" : "drawing";
	document.getElementById("modeToggle").textContent =
		mode === "drawing" ? "Switch to Driving Mode" : "Switch to Drawing Mode";
	draw();
});
