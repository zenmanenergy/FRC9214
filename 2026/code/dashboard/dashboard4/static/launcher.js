function openDashboard() {
	// Open dashboard
	const dashboardWindow = window.open("index.html", "_blank",
		`width=${screen.availWidth},height=${screen.availHeight},left=0,top=0,resizable=yes`
	);

	// Request fullscreen (only works when triggered by user input)
	if (dashboardWindow) {
		setTimeout(() => {
			dashboardWindow.document.documentElement.requestFullscreen();
		}, 1000);
	} else {
		alert("⚠️ Popup blocked! Please allow popups.");
	}
}



// Run when page loads
window.onload = openDashboard;
