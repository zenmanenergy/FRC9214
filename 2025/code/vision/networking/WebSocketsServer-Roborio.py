import wpilib
import asyncio
import websockets
from threading import Thread


class MyRobot(wpilib.TimedRobot):

	def robotInit(self):
		# Initialize any robot components here
		wpilib.SmartDashboard.putString("Status", "Robot Initialized")

		# Start the WebSocket server in a separate thread
		self.websocketThread = Thread(target=self.runWebSocketServer, daemon=True)
		self.websocketThread.start()

	def runWebSocketServer(self):
		async def server(websocket, path):
			try:
				async for message in websocket:
					# Log the message to the FRC Driver Station
					wpilib.DriverStation.reportWarning(f"Received: {message}", False)
			except Exception as e:
				wpilib.DriverStation.reportError(f"WebSocket Error: {e}", False)

		# Create the WebSocket server
		start_server = websockets.serve(server, "0.0.0.0", 5800)

		# Run the WebSocket server
		asyncio.set_event_loop(asyncio.new_event_loop())
		asyncio.get_event_loop().run_until_complete(start_server)
		asyncio.get_event_loop().run_forever()

	def teleopPeriodic(self):
		# Add any periodic teleop code here
		wpilib.SmartDashboard.putString("Status", "Teleop Running")

	def autonomousPeriodic(self):
		# Add any periodic autonomous code here
		wpilib.SmartDashboard.putString("Status", "Autonomous Running")


if __name__ == "__main__":
	wpilib.run(MyRobot)
