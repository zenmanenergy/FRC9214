function saveData() {
	localStorage.setItem("dots", JSON.stringify(dots));
	localStorage.setItem("connections", JSON.stringify(connections));
}

function loadData() {
	const savedDots = localStorage.getItem("dots");
	const savedConnections = localStorage.getItem("connections");

	if (savedDots) dots = JSON.parse(savedDots);
	if (savedConnections) connections = JSON.parse(savedConnections);
}

loadData();
