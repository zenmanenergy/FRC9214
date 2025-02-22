from flask import Flask, render_template
from flask_socketio import SocketIO
from networktables import NetworkTables
import time
import threading

# Flask Setup
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading", logger=False, engineio_logger=False)


# Connect to NetworkTables
roborio_ip = "192.168.1.193"  # Change if needed
NetworkTables.initialize(server=roborio_ip)
table = NetworkTables.getTable("arm")

@app.route("/")
def index():
	return render_template("index.html")

# Periodically send NetworkTables updates
def broadcast_data():
	while True:
		socketio.sleep(0.1)  # Reduce delay (was 1 second)
		data = {
			"elevator": table.getNumber("elevator", 0),
		}
		print(f"📡 Sending data: {data}")
		socketio.emit("update_data", data)

# Start broadcasting in Flask's event loop (NON-blocking)
socketio.start_background_task(broadcast_data)


# Start broadcasting in the background
data_thread = threading.Thread(target=broadcast_data, daemon=True)
data_thread.start()

@socketio.on("send_command")
def receive_command(command):
	print(f"🛠 Received command from browser: {command}")
	table.putString("dashboard_command", command)

if __name__ == "__main__":
	print("🚀 Flask Server Running on http://0.0.0.0:5805")
	socketio.run(app, host="0.0.0.0", port=5805)
