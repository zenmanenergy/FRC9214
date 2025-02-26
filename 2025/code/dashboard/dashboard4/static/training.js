// Ensure these elements exist
const positionNameInput = document.getElementById("positionName");
const savedPositionsList = document.getElementById("savedPositions");


function animateMove(targetPos) {
	const steps = 20;
	let step = 0;

	const start = {
		elevatorHeight,
		armAngle,
		wristAngle,
		grabberAngle,
	};

	function interpolate(startValue, endValue) {
		return startValue + (endValue - startValue) * (step / steps);
	}

	function stepMove() {
		if (step >= steps) {
			elevatorHeight = targetPos.elevatorHeight;
			armAngle = Math.max(LIMITS.ARM_MIN, Math.min(LIMITS.ARM_MAX, targetPos.armAngle));
			wristAngle = Math.max(LIMITS.WRIST_MIN, Math.min(LIMITS.WRIST_MAX, targetPos.wristAngle));
			grabberAngle = Math.max(LIMITS.GRABBER_MIN, Math.min(LIMITS.GRABBER_MAX, targetPos.grabberAngle)); // Use LIMITS
			draw();
			return;
		}

		elevatorHeight = interpolate(start.elevatorHeight, targetPos.elevatorHeight);
		armAngle = Math.max(LIMITS.ARM_MIN, Math.min(LIMITS.ARM_MAX, interpolate(start.armAngle, targetPos.armAngle)));
		wristAngle = Math.max(LIMITS.WRIST_MIN, Math.min(LIMITS.WRIST_MAX, interpolate(start.wristAngle, targetPos.wristAngle)));
		grabberAngle = Math.max(LIMITS.GRABBER_MIN, Math.min(LIMITS.GRABBER_MAX, interpolate(start.grabberAngle, targetPos.grabberAngle)));

		draw();
		step++;
		setTimeout(stepMove, 50);
	}

	stepMove();
}
function savePosition() {
	const name = positionNameInput.value.trim();
	if (!name) {
		console.log("No name entered for position!");
		return;
	}

	const positions = JSON.parse(localStorage.getItem("robotArmPositions") || "[]");

	// Check if we're actually saving
	console.log(`Saving position: ${name}, Values:`, { 
		elevatorHeight, armAngle, wristAngle, grabberAngle 
	});

	// Add the new position
	positions.push({ name, elevatorHeight, armAngle, wristAngle, grabberAngle });
	localStorage.setItem("robotArmPositions", JSON.stringify(positions));

	// Reload saved positions in the dropdown
	loadSavedPositions();
}

function loadSavedPositions() {
	const positions = JSON.parse(localStorage.getItem("robotArmPositions") || "[]");
	savedPositionsList.innerHTML = "";

	positions.forEach((pos, index) => {
		const li = document.createElement("li");
		li.textContent = pos.name + " ";

		const moveBtn = document.createElement("button");
		moveBtn.textContent = "Move";
		moveBtn.onclick = () => animateMove(pos);
		li.appendChild(moveBtn);

		const deleteBtn = document.createElement("button");
		deleteBtn.textContent = "Delete";
		deleteBtn.classList.add("delete");
		deleteBtn.onclick = () => {
			positions.splice(index, 1);
			localStorage.setItem("robotArmPositions", JSON.stringify(positions));
			loadSavedPositions();
		};
		li.appendChild(deleteBtn);

		savedPositionsList.appendChild(li);
	});

	// Ensure UI updates after loading positions, but only if updateModeUI is defined
	if (typeof updateModeUI === "function") {
		updateModeUI();
	}
}



// Ensure saved positions are loaded on startup
loadSavedPositions();
