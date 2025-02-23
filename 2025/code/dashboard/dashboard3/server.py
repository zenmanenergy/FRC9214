from flask import Flask, render_template
from flask_socketio import SocketIO
from networktables import NetworkTables
import threading

# Flask Setup
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading", logger=False, engineio_logger=False)

# Connect to NetworkTables
roborio_ip = "192.168.1.125"  # Change if needed
NetworkTables.initialize(server=roborio_ip)
table = NetworkTables.getTable("robot_data")

@app.route("/")
def index():
	return render_template("index.html")

# Periodically send NetworkTables updates to the dashboard
def broadcast_data():
	while True:
		socketio.sleep(0.1)  # 10Hz refresh rate
		data = {
			"x_position": table.getNumber("x_position", 0),
			"y_position": table.getNumber("y_position", 0),
			"elevator": table.getNumber("elevator", 0),
			"arm_angle": table.getNumber("arm_angle", 0),
			"wrist_angle": table.getNumber("wrist_angle", 0),
			"grabber_angle": table.getNumber("grabber_angle", 0),
		}
		socketio.emit("update_data", data)

# Start broadcasting data in the background
socketio.start_background_task(broadcast_data)

@socketio.on("send_command")
def receive_command(command):
	""" Receives commands from the dashboard and forwards them to NetworkTables """
	for key, value in command.items():
		table.putNumber(key, value)  # Store commands in NetworkTables

if __name__ == "__main__":
	print("🚀 Flask Server Running on http://0.0.0.0:5805")
	socketio.run(app, host="0.0.0.0", port=5805)
