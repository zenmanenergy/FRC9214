from adafruit_servokit import ServoKit
import busio
from board import SCL, SDA
import atexit
import time
from pynput import keyboard

# Initialize PCA9685 and ServoKit
i2c_bus = busio.I2C(SCL, SDA)
kit = ServoKit(channels=16)

# Set initial angles for each servo
angles = {0: 90, 1: 90, 2: 90, 3: 90, 4: 90, 5: 90}
step_size = 5
min_angle = 0
max_angle = 180

# Function to move servo and release torque
def adjust_servo(servo, direction):
	print(f"Debug: adjust_servo called for servo {servo} in direction {direction}")

	if servo not in angles:
		print(f"Debug: Invalid servo number {servo}.")
		return
	
	initial_angle = angles[servo]
	
	# Update angle based on direction
	if direction == 'up' and angles[servo] < max_angle:
		angles[servo] += step_size
	elif direction == 'down' and angles[servo] > min_angle:
		angles[servo] -= step_size

	# Move servo to the updated angle
	kit.servo[servo].angle = angles[servo]
	print(f"Moved servo {servo} from {initial_angle} to {angles[servo]}")
	time.sleep(0.5)  # Allow time for movement

	# Release torque to prevent overheating
	kit.servo[servo].angle = None
	print(f"Torque released for servo {servo}")

# Function to stop all servos on exit
def stop_all_servos():
	print("Stopping all servos on program exit...")
	for i in range(6):  # Adjust the range if needed
		kit.servo[i].angle = None
	print("All servos stopped.")

# Register the cleanup function to run on exit
atexit.register(stop_all_servos)

# Set all servos to initial angle of 90 at the start of the program
print("Setting all servos to initial angle of 90 degrees...")
for servo in angles:
	kit.servo[servo].angle = angles[servo]
	print(f"Debug: Servo {servo} set to 90")
time.sleep(1)  # Allow time for servos to reach 90 degrees
stop_all_servos()  # Release torque after setting initial position

# Key press handling
def on_press(key):
	try:
		print(f"Debug: Key {key.char} pressed.")
		
		if key.char == 'w':      # Move eyes up
			adjust_servo(1, 'up')
		elif key.char == 's':    # Move eyes down
			adjust_servo(1, 'down')
		elif key.char == 'a':    # Move eyes left
			adjust_servo(0, 'up')
		elif key.char == 'd':    # Move eyes right
			adjust_servo(0, 'down')
		elif key.char == 'q':    # Left upper eyelid up
			adjust_servo(2, 'down')
		elif key.char == 'z':    # Left upper eyelid down
			adjust_servo(2, 'up')
		elif key.char == 'e':    # Left lower eyelid up
			adjust_servo(3, 'down')
		elif key.char == 'c':    # Left lower eyelid down
			adjust_servo(3, 'up')
		elif key.char == 'r':    # Right upper eyelid up
			adjust_servo(4, 'up')
		elif key.char == 'v':    # Right upper eyelid down
			adjust_servo(4, 'down')
		elif key.char == 't':    # Right lower eyelid up
			adjust_servo(5, 'up')
		elif key.char == 'b':    # Right lower eyelid down
			adjust_servo(5, 'down')
	except AttributeError:
		print(f"Debug: Non-character key {key} pressed.")

def main():
	print("Press keys to control servos. Press ESC to quit.")
	with keyboard.Listener(on_press=on_press) as listener:
		listener.join()

# Run the main function
main()

