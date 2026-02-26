#!/usr/bin/env python3

import os
import http.server
import socketserver

# Get the client directory
CLIENT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'client')

# Change to the client directory
os.chdir(CLIENT_DIR)

PORT = 8000
Handler = http.server.SimpleHTTPRequestHandler

print(f"Server running at http://127.0.0.1:{PORT}")
print(f"Access index.html at: http://127.0.0.1:{PORT}/index.html")
print(f"Serving files from: {CLIENT_DIR}")
print("Press Ctrl+C to stop\n")

with socketserver.TCPServer(("", PORT), Handler) as httpd:
	httpd.serve_forever()
