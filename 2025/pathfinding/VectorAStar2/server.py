import asyncio
import websockets
import json

robot_state = {"x": 100, "y": 100, "heading": 0}  # Initial position of the actual robot
connected_clients = set()

async def handle_connection(websocket, path):
	global robot_state
	connected_clients.add(websocket)
	try:
		async for message in websocket:
			try:
				data = json.loads(message)
				if "path" in data:
					print("Received path:", data["path"])  # Print received path
				elif "get_state" in data:  # If the client requests the robot state
					await websocket.send(json.dumps(robot_state))
			except json.JSONDecodeError:
				print("Invalid JSON received")
	finally:
		connected_clients.remove(websocket)

async def broadcast_robot_state():
	while True:
		if connected_clients:
			message = json.dumps(robot_state)
			await asyncio.gather(*[client.send(message) for client in connected_clients if not client.closed])
		await asyncio.sleep(0.1)  # Send updates every 100ms

async def main():
	server = await websockets.serve(handle_connection, "localhost", 8765)
	print("WebSocket server started on ws://localhost:8765")
	await asyncio.gather(server.wait_closed(), broadcast_robot_state())

if __name__ == "__main__":
	asyncio.run(main())