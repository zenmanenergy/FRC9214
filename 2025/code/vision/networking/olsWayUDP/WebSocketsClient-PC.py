import asyncio
import websockets

async def send_hello():
	async with websockets.connect("ws://10.92.14.2:5800") as websocket:  # Replace 10.92.14.2 with your RoboRIO's IP
		message = "hello world"
		await websocket.send(message)
		print(f"Sent: {message}")

asyncio.run(send_hello())
