document.getElementById("scrapeBtn").addEventListener("click", () => {
	chrome.tabs.query({ active: true, currentWindow: true }, (tabs) => {
		chrome.scripting.executeScript({
			target: { tabId: tabs[0].id },
			func: scrapeAllGrantWatchPages
		});
	});
});

function scrapeAllGrantWatchPages() {
	let all_grants = [];
	let total_pages = 92;
	let current_page = 1;

	function fetchAndScrapePage(page_num) {
		let url = `https://www.grantwatch.com/cat/59/education-grants.html/${page_num}`;
		fetch(url)
			.then(response => response.text())
			.then(html => {
				let parser = new DOMParser();
				let doc = parser.parseFromString(html, "text/html");
				let listings = doc.querySelectorAll(".grnhomegbox");

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

					all_grants.push({ title, url, description, deadline, grant_id });
				});

				console.log(`✅ Scraped page ${page_num}, total grants so far: ${all_grants.length}`);

				if (page_num < total_pages) {
					setTimeout(() => {
						fetchAndScrapePage(page_num + 1);
					}, 1000);
				} else {
					downloadCSV();
				}
			})
			.catch(err => {
				console.error(`Error loading page ${page_num}`, err);
			});
	}

	function downloadCSV() {
		let csv = "Title,URL,Description,Deadline,GrantWatchID\n";
		all_grants.forEach(grant => {
			let row = `"${grant.title.replace(/"/g, '""')}","${grant.url}","${grant.description.replace(/"/g, '""')}","${grant.deadline}","${grant.grant_id}"\n`;
			csv += row;
		});
		let blob = new Blob([csv], { type: "text/csv" });
		let link = document.createElement("a");
		link.href = URL.createObjectURL(blob);
		link.download = "grantwatch_all_pages.csv";
		link.click();
	}

	fetchAndScrapePage(current_page);
}
