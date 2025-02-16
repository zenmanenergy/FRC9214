import wpilib
import socket

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		"""Initialization code runs once when the robot powers on."""
		# Set up a UDP socket to listen for messages on port 5800
		self.udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.udpSocket.bind(("0.0.0.0", 5800))  # Listen on all interfaces at port 5800
		self.udpSocket.setblocking(False)
		print("UDP Server listening on port 5800")

	def teleopPeriodic(self):
		"""Periodic code that runs during teleoperated control."""
		try:
			# Receive data (1024-byte buffer size)
			data, addr = self.udpSocket.recvfrom(1024)

			# Decode and process the message
			message = data.decode()
			# wpilib.DriverStation.reportWarning(f"Received: {message}", False)
			print(f"Received: {message} from {addr}")

		except BlockingIOError:
			# No data received, keep looping
			pass
		except Exception as e:
			# Log any errors
			# wpilib.DriverStation.reportError(f"UDP Error: {e}", False)
			print("udp error")

if __name__ == "__main__":
	wpilib.run(MyRobot)
