from flask import Flask, render_template
from flask_socketio import SocketIO
from networktables import NetworkTables
import threading
import time
from vision import vision_loop  # Import the vision processing module

# Flask Setup
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading", logger=False, engineio_logger=False)

# Connect to NetworkTables
roborio_ip = "roborio-9214-frc.local"  # Update with actual RoboRIO IP
NetworkTables.initialize(server=roborio_ip)
table = NetworkTables.getTable("robot_data")

# Wait for NetworkTables to connect
time.sleep(1)  # Small delay to allow connection

if NetworkTables.isConnected():
	print("Connected to NetworkTables on", roborio_ip)
else:
	print("Failed to connect to NetworkTables. Check IP and network.")

@app.route("/")
def index():
	return render_template("index.html")

@socketio.on("connect")
def handle_connect():
	print("Browser WebSocket Connected")

@socketio.on("send_command")
def receive_command(command):
	print("Received command from browser:", command)
	table.putString("dashboard_command", command)

# ✅ Periodically send NetworkTables updates to browser (EXCLUDING vision data)
def broadcast_data():
	while True:
		socketio.sleep(0.1)  # Update 10 times per second

		# Read real values from NetworkTables (from RoboRIO)
		data = {
			"real_x_position": table.getNumber("real_x_position", 0),  # FROM ROBO-RIO
			"real_y_position": table.getNumber("real_y_position", 0),  # FROM ROBO-RIO
			"real_heading": table.getNumber("real_heading", 0),  # FROM ROBO-RIO
			"real_elevator": table.getNumber("real_elevator", 0),
			"real_arm_angle": table.getNumber("real_arm_angle", 0),
			"real_wrist_angle": table.getNumber("real_wrist_angle", 0),
			"real_grabber_angle": table.getNumber("real_grabber_angle", 0),
		}

		# ✅ Broadcast data, excluding vision data
		socketio.emit("update_data", data)

# ✅ Start Vision Processing in a Background Thread (Updates NetworkTables Every 5 Secs)
vision_thread = threading.Thread(target=vision_loop, daemon=True)
vision_thread.start()

# ✅ Start broadcasting data (but NOT vision data) in the background
socketio.start_background_task(broadcast_data)

if __name__ == "__main__":
	print("Starting Flask Server on http://0.0.0.0:5805")
	socketio.run(app, host="0.0.0.0", port=5805)
