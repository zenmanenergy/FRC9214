from controller import Robot
import socket
import struct
import time  # Import time for delay

TIME_STEP = 32
robot = Robot()

# Match the motor names exactly from the .proto
left_front = robot.getDevice("left_front_joint")
left_rear = robot.getDevice("left_rear_joint")
right_front = robot.getDevice("right_front_joint")
right_rear = robot.getDevice("right_rear_joint")

for m in [left_front, left_rear, right_front, right_rear]:
	m.setPosition(float('inf'))
	m.setVelocity(0)

# Networking setup
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('127.0.0.1', 9999))
server.listen(1)
server.settimeout(1.0)  # 1 second timeout for accept

print("Waiting for WPILib simulation...", flush=True)
conn = None
while robot.step(TIME_STEP) != -1 and conn is None:
    try:
        conn, addr = server.accept()
        print("Connected from", addr)
        conn.settimeout(0.1)  # Non-blocking recv with short timeout
    except socket.timeout:
        continue

if conn is None:
    print("Controller terminated before connection established.")
    server.close()
    exit(0)

print("Initializing controller...")
time.sleep(1)  # 1-second delay
print("Controller initialized. Starting main loop.")

try:
    while robot.step(TIME_STEP) != -1:
        left = 0.0
        right = 0.0
        data_received = False
        try:
            data = conn.recv(8)
            if data and len(data) == 8:
                left, right = struct.unpack("ff", data)
                data_received = True
        except socket.timeout:
            pass
        except Exception as e:
            print("Error during communication:", e)
            break

        # Always set velocities, even if no data received (default to zero)
        left_front.setVelocity(left)
        left_rear.setVelocity(left)
        right_front.setVelocity(right)
        right_rear.setVelocity(right)
except KeyboardInterrupt:
    print("Terminating controller...")
finally:
    if conn:
        conn.close()
    server.close()
    print("Controller terminated.")
