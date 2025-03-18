import csv

input_file = 'c:\\Apache24\\htdocs\\personalProjects\\FRC9214\\2025\\grants\\grantwatch_scraper\\grant_data.csv'
output_file = 'c:\\Apache24\\htdocs\\personalProjects\\FRC9214\\2025\\grants\\grantwatch_scraper\\grant_data.js'

with open(input_file, 'r', encoding='utf-8') as csvfile:
    lines = csvfile.readlines()[1:]  # Skip header row

grants = [f"'{line.strip()}'" for line in lines]

with open(output_file, 'w', encoding='utf-8') as jsfile:
    jsfile.write('const grantCSV = [\n')
    jsfile.write(',\n'.join(grants))
    jsfile.write('\n];\n')
