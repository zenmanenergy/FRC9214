import cv2
import mediapipe as mp
import time
from adafruit_servokit import ServoKit
import busio
from board import SCL, SDA
import atexit

# Initialize the PCA9685 and ServoKit
i2c_bus = busio.I2C(SCL, SDA)
kit = ServoKit(channels=16)

# Initialize mediapipe face mesh modules
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils

# Set up the webcam
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

# Initialize the FaceMesh model
with mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5) as face_mesh:

	# Set initial angles for servos
	angles = {0: 90, 1: 90, 2: 90, 3: 90}  # Servo 0 and 1 for horizontal, 2 and 3 for vertical movement
	min_angle = 0
	max_angle = 180

	# Function to adjust servo based on yaw/pitch
	def adjust_eye_servos(yaw, pitch):
		# Map yaw to servo angles for left-right eye movement (servo 0 and 1)
		yaw_angle = 90 + (yaw / 90 * 90)  # Scale yaw to the servo range
		yaw_angle = max(min(yaw_angle, max_angle), min_angle)  # Clamp to valid servo range

		# Map pitch to servo angles for up-down eye movement (servo 2 and 3)
		pitch_angle = 90 - (pitch / 90 * 90)  # Scale pitch to the servo range (inverted)
		pitch_angle = max(min(pitch_angle, max_angle), min_angle)  # Clamp to valid servo range

		# Move servos based on calculated yaw/pitch
		kit.servo[0].angle = yaw_angle  # Horizontal left-right movement (left eye)
		kit.servo[1].angle = yaw_angle  # Horizontal left-right movement (right eye)
		kit.servo[2].angle = pitch_angle  # Vertical up-down movement (left eye)
		kit.servo[3].angle = pitch_angle  # Vertical up-down movement (right eye)
		print(f"Moved servos to Yaw: {yaw_angle}, Pitch: {pitch_angle}")

	# Function to stop all servos on exit
	def stop_all_servos():
		print("Stopping all servos on program exit...")
		for i in range(4):  # Adjust the range if needed (4 servos in total)
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

	while True:
		# Read a frame from the camera
		ret, frame = cap.read()
		if not ret:
			break

		# Flip the frame horizontally for a more natural view
		frame = cv2.flip(frame, 1)

		# Convert to RGB as mediapipe uses RGB images
		rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

		# Process the frame with Face Mesh to get facial landmarks
		mesh_results = face_mesh.process(rgb_frame)

		# If face landmarks are found, calculate yaw and pitch and adjust servos
		if mesh_results.multi_face_landmarks:
			for face_landmarks in mesh_results.multi_face_landmarks:
				ih, iw, _ = frame.shape

				# Initialize extreme values for the bounding box
				min_x, min_y, max_x, max_y = iw, ih, 0, 0

				# Iterate through the landmarks to find the min/max coordinates
				for landmark in face_landmarks.landmark:
					x = int(landmark.x * iw)
					y = int(landmark.y * ih)

					min_x = min(min_x, x)
					min_y = min(min_y, y)
					max_x = max(max_x, x)
					max_y = max(max_y, y)

					# Draw landmarks
					cv2.circle(frame, (x, y), 1, (255, 0, 0), -1)

				# Calculate the center of the face for yaw and pitch calculation
				face_center_x = min_x + (max_x - min_x) // 2
				face_center_y = min_y + (max_y - min_y) // 2

				# Calculate the yaw and pitch based on face position in the frame
				center_x = iw // 2
				center_y = ih // 2

				# Calculate yaw and pitch
				yaw = (face_center_x - center_x) / center_x * 90  # -90 to 90 range
				pitch = -(face_center_y - center_y) / center_y * 90  # -90 to 90 range

				# Display yaw and pitch values on the frame
				cv2.putText(frame, f"Yaw: {yaw:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
				cv2.putText(frame, f"Pitch: {pitch:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

				# Adjust servos based on yaw and pitch
				adjust_eye_servos(yaw, pitch)

		# Display the result
		cv2.imshow('Face Detection with Yaw, Pitch, and Animatronic Eyes', frame)

		# Exit on pressing 'q'
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
