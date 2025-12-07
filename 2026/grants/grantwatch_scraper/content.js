if (localStorage.getItem("grant_scraper_active") === "true") {
	window.addEventListener("load", () => {
		setTimeout(() => {
			scrapeAndContinue();
		}, 1500);
	});
}

function scrapeAndContinue() {
	let data = JSON.parse(localStorage.getItem("grant_scraper_data") || "[]");

	let listings = document.querySelectorAll(".grnhomegbox");
	listings.forEach(box => {
		let aTag = box.querySelector("a[href*='/grant/']");
		let h4 = box.querySelector("h4");
		let desc = box.querySelector(".grnhomegboxtext p");
		let deadlineTag = box.querySelector(".ddlinedtgwhm em");
		let grantIdTag = box.querySelector(".gridgwhm span");

		let title = h4?.innerText.trim() || "";
		let url = aTag?.href || "";
		let description = desc?.innerText.trim() || "";
		let deadline = deadlineTag?.innerText.trim() || "";
		let grant_id = grantIdTag?.innerText.trim() || "";

		data.push({ title, url, description, deadline, grant_id });
	});

	localStorage.setItem("grant_scraper_data", JSON.stringify(data));

	let nextLink = document.querySelector("ul.pagination li a[aria-label='Next']");
	let nextDisabled = nextLink?.parentElement?.classList.contains("disabled");

	if (nextLink && !nextDisabled) {
		console.log("➡ Clicking next page...");
		setTimeout(() => {
			nextLink.click(); // navigate forward, content.js will run again
		}, 1000);
	} else {
		console.log("✅ No more pages. Downloading CSV...");

		let csv = "Title,URL,Description,Deadline,GrantWatchID\n";
		data.forEach(grant => {
			let row = `"${grant.title.replace(/"/g, '""')}","${grant.url}","${grant.description.replace(/"/g, '""')}","${grant.deadline}","${grant.grant_id}"\n`;
			csv += row;
		});

		let blob = new Blob([csv], { type: "text/csv" });
		let link = document.createElement("a");
		link.href = URL.createObjectURL(blob);
		link.download = "grantwatch_all_pages.csv";
		link.click();

		localStorage.removeItem("grant_scraper_data");
		localStorage.removeItem("grant_scraper_active");
	}
}
