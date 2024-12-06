import cv2
import mediapipe as mp
import time
import json
import numpy as np

# Initialize mediapipe face mesh modules
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils

# Set up the webcam
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Initialize the FaceMesh model
with mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5) as face_mesh:

	# Load or create a dictionary to store personal features
	try:
		with open('personal_features.json', 'r') as f:
			personal_data = json.load(f)
	except FileNotFoundError:
		personal_data = {}

	def countdown(seconds):
		"""Function to display countdown"""
		for i in range(seconds, 0, -1):
			cv2.putText(frame, f"Next in: {i}s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
			cv2.imshow('Face Detection with Yaw/Pitch', frame)
			cv2.waitKey(1000)  # wait for 1 second

	def capture_person():
		"""Capture and store personal features after asking for their name"""
		# Countdown before capturing the person's features
		countdown(3)
		
		# Ask for the person's name
		name = input("Please enter your name: ")
		
		# Capture facial landmarks and yaw/pitch for the person
		person_landmarks = []
		for _ in range(5):  # Capture multiple frames for averaging
			ret, frame = cap.read()
			if not ret:
				break
			frame = cv2.flip(frame, 1)
			rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
			mesh_results = face_mesh.process(rgb_frame)

			if mesh_results.multi_face_landmarks:
				for face_landmarks in mesh_results.multi_face_landmarks:
					# Store landmarks for the face
					landmarks = []
					for landmark in face_landmarks.landmark:
						landmarks.append((landmark.x, landmark.y, landmark.z))
					person_landmarks.append(np.array(landmarks))
					
					# Calculate the center of the face for yaw and pitch calculation
					ih, iw, _ = frame.shape
					min_x, min_y, max_x, max_y = iw, ih, 0, 0

					# Iterate through the landmarks to find the min/max coordinates
					for landmark in face_landmarks.landmark:
						x = int(landmark.x * iw)
						y = int(landmark.y * ih)

						min_x = min(min_x, x)
						min_y = min(min_y, y)
						max_x = max(max_x, x)
						max_y = max(max_y, y)

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

			cv2.imshow('Face Detection with Yaw/Pitch', frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		
		# Store the average of the landmarks and yaw/pitch for the person
		if person_landmarks:
			avg_landmarks = np.mean(person_landmarks, axis=0)
			personal_data[name] = {
				'landmarks': avg_landmarks.tolist(),
				'yaw_pitch': (yaw, pitch)
			}

	def recognize_person():
		"""Recognize a person based on stored features"""
		ret, frame = cap.read()
		if not ret:
			return None, frame
		frame = cv2.flip(frame, 1)
		rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		mesh_results = face_mesh.process(rgb_frame)

		recognized_names = []

		if mesh_results.multi_face_landmarks:
			for face_landmarks in mesh_results.multi_face_landmarks:
				ih, iw, _ = frame.shape
				min_x, min_y, max_x, max_y = iw, ih, 0, 0

				# Iterate through the landmarks to find the min/max coordinates
				for landmark in face_landmarks.landmark:
					x = int(landmark.x * iw)
					y = int(landmark.y * ih)

					min_x = min(min_x, x)
					min_y = min(min_y, y)
					max_x = max(max_x, x)
					max_y = max(max_y, y)

				# Calculate the center of the face for yaw and pitch calculation
				face_center_x = min_x + (max_x - min_x) // 2
				face_center_y = min_y + (max_y - min_y) // 2

				# Calculate the yaw and pitch based on face position in the frame
				center_x = iw // 2
				center_y = ih // 2

				# Calculate yaw and pitch
				yaw = (face_center_x - center_x) / center_x * 90  # -90 to 90 range
				pitch = -(face_center_y - center_y) / center_y * 90  # -90 to 90 range

				# Compare the current yaw/pitch with stored data
				for name, data in personal_data.items():
					stored_yaw, stored_pitch = data['yaw_pitch']
					if abs(yaw - stored_yaw) < 15 and abs(pitch - stored_pitch) < 15:
						# Draw the name on the person's forehead
						cv2.putText(frame, name, (min_x, min_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
						recognized_names.append(name)
		
		return recognized_names, frame

	while True:
		# Look for new person or recognize existing ones
		recognized_names, frame = recognize_person()
		
		if not recognized_names:
			# If no person recognized, ask for new person
			cv2.putText(frame, "New person detected, get ready!", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
			countdown(3)
			capture_person()
		
		cv2.imshow('Face Detection with Yaw/Pitch', frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	# Save the personal data to a file
	with open('personal_features.json', 'w') as f:
		json.dump(personal_data, f)

	cap.release()
	cv2.destroyAllWindows()
	print("Personal features saved to 'personal_features.json'")
