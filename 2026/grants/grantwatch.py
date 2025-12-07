from playwright.sync_api import sync_playwright
import pandas as pd
import os
import time

all_grants = []
max_pages = 100  # Stop if we hit a page with no data

def scrape_page(page, browser):
	print(f"Scraping page {page}...")
	url = f"https://www.grantwatch.com/cat/59/education-grants.html/{page}"
	context = browser.new_context()
	page_obj = context.new_page()
	page_obj.goto(url)
	page_obj.wait_for_selector('.grantCard', timeout=10000)

	grant_cards = page_obj.query_selector_all('.grantCard')
	if not grant_cards:
		print("No grant cards found.")
		return False

	for card in grant_cards:
		try:
			title_tag = card.query_selector('.grantTitle')
			desc_tag = card.query_selector('.grantDescription')
			title = title_tag.inner_text().strip()
			link = title_tag.get_attribute('href')
			description = desc_tag.inner_text().strip() if desc_tag else ''
			full_link = f"https://www.grantwatch.com{link}" if link.startswith('/') else link
			all_grants.append({
				"url": full_link,
				"title": title,
				"description": description
			})
		except Exception as e:
			print(f"Skipping a card: {e}")
	page_obj.close()
	return True

def main():
	with sync_playwright() as p:
		browser = p.chromium.launch(headless=True)
		for page_num in range(1, max_pages + 1):
			success = scrape_page(page_num, browser)
			if not success:
				break
			time.sleep(1)  # polite pause
		browser.close()

	script_dir = os.path.dirname(os.path.abspath(__file__))
	csv_path = os.path.join(script_dir, "grantwatch_grants.csv")
	df = pd.DataFrame(all_grants)
	df.to_csv(csv_path, index=False)
	print(f"✅ Done! Saved {len(df)} grants to {csv_path}")

if __name__ == "__main__":
	main()
