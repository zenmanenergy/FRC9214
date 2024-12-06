import cv2
import os
import time
import mediapipe as mp
import json
import numpy as np

def capture_expression():
	# Get the current directory where the .py file is located
	current_dir = os.path.dirname(os.path.abspath(__file__))
	
	# Define the path for the 'gestures' folder relative to the current directory
	gestures_dir = os.path.join(current_dir, 'gestures')

	# Set up webcam
	cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

	# Check if the webcam is opened
	if not cap.isOpened():
		print("Error: Could not open webcam.")
		return

	# Create the gestures directory if it doesn't exist
	if not os.path.exists(gestures_dir):
		os.makedirs(gestures_dir)

	# Initialize MediaPipe FaceMesh
	mp_face_mesh = mp.solutions.face_mesh
	face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
	mp_drawing = mp.solutions.drawing_utils

	# Customize the drawing of the landmarks to mimic the example code
	drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1, color=(255, 0, 0))  # Blue color and small dots

	# Initialize dictionary to hold landmark data
	landmark_data = {}

	# List of expressions to capture
	expressions = ['smile', 'frown', 'surprised']

	# Define the landmarks that are relevant for expressions (eyes, mouth, eyebrows)
	relevant_landmarks = [
		# Eyes (left and right eyes, around the eye area)
		33, 133, 160, 158, 144, 145, 159, 130, 243, 473, 474, 263,
		# Eyebrows (for furrowed brows, surprise)
		70, 63, 105, 66, 55, 285, 284, 300, 296,
		# Nose (for expressions involving nose movements)
		1, 2, 4, 6, 168, 197,
		# Mouth (for smile/frown)
		61, 185, 317, 402, 81, 375, 291, 56, 185, 85, 312, 97, 91
	]

	for expression in expressions:
		print(f"Capturing expression: {expression.capitalize()}")
		
		# Initialize a list to store data for each expression
		expression_data = []
		
		# Number of images to capture per expression
		num_images = 5
		
		for i in range(num_images):
			# Display the video feed and countdown for each expression
			countdown_time = 3
			start_time = time.time()

			while time.time() - start_time < countdown_time:
				ret, frame = cap.read()
				if not ret:
					print("Error: Failed to capture frame.")
					break

				# Calculate remaining time for countdown
				remaining_time = countdown_time - int(time.time() - start_time)

				# Display the countdown and the expression text
				cv2.putText(frame, f"{expression.capitalize()} in {remaining_time} seconds", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
				cv2.imshow('Expression Detection', frame)

				# Wait for 1 ms to update the frame without freezing
				cv2.waitKey(1)

			# After countdown, capture the image
			ret, frame = cap.read()
			if ret:
				# Convert the frame to RGB for MediaPipe processing
				rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
				
				# Process the frame to detect facial landmarks
				results = face_mesh.process(rgb_frame)

				# If landmarks are detected, draw them on the frame and store the data
				if results.multi_face_landmarks:
					for idx, face_landmarks in enumerate(results.multi_face_landmarks):
						# Create a list to hold the landmarks for this face
						landmarks_list = []
						
						# Store only relevant landmarks for expression detection
						for i in relevant_landmarks:
							# Ensure the index exists in the face_landmarks
							if i < len(face_landmarks.landmark):
								landmark = face_landmarks.landmark[i]
								landmarks_list.append({'x': landmark.x, 'y': landmark.y, 'z': landmark.z})

						# Add the landmarks for this image to the expression's data
						expression_data.append(landmarks_list)

						# Draw landmarks on the frame (smaller blue dots)
						for i in relevant_landmarks:  # Loop through the relevant landmarks
							if i < len(face_landmarks.landmark):  # Check if the index is within the range
								landmark = face_landmarks.landmark[i]
								x = int(landmark.x * frame.shape[1])
								y = int(landmark.y * frame.shape[0])
								cv2.circle(frame, (x, y), 1, (255, 0, 0), -1)  # Blue dot with size 1

				# Save the image with landmarks
				img_filename = os.path.join(gestures_dir, f'{expression}_{i+1}_{time.strftime("%Y%m%d-%H%M%S")}.jpg')
				cv2.imwrite(img_filename, frame)
				print(f"Image saved as {img_filename}")

		# Store all images' landmark data for this expression
		landmark_data[expression] = expression_data
		
		# Save the facial landmark data to a JSON file without whitespace
		json_filename = os.path.join(gestures_dir, 'expressions.json')
		with open(json_filename, 'w') as json_file:
			json.dump(landmark_data, json_file, separators=(',', ':'))  # No extra whitespace
		print(f"Expression data saved as {json_filename}")

	# Clean up and release resources
	cap.release()
	cv2.destroyAllWindows()


def get_head_pose(landmarks):
	"""Estimate the head pose using the position of specific facial landmarks"""
	# Nose tip (landmark 1) and eyes (landmarks 33, 133)
	nose_tip = np.array([landmarks[1].x, landmarks[1].y, landmarks[1].z])
	left_eye = np.array([landmarks[33].x, landmarks[33].y, landmarks[33].z])
	right_eye = np.array([landmarks[133].x, landmarks[133].y, landmarks[133].z])

	# Calculate the head pose (using a simple estimation from the nose and eyes)
	# Simplified logic for head pose estimation
	vector_nose_eye = right_eye - left_eye
	yaw = np.arctan2(vector_nose_eye[1], vector_nose_eye[0]) * (180 / np.pi)
	pitch = np.arctan2(vector_nose_eye[2], np.linalg.norm(vector_nose_eye[:2])) * (180 / np.pi)
	roll = 0  # Assume roll is 0 for simplicity

	return yaw, pitch, roll

def predict_expression():
	# Get the current directory where the .py file is located
	current_dir = os.path.dirname(os.path.abspath(__file__))
	gestures_dir = os.path.join(current_dir, 'gestures')

	# Check if the expressions.json file exists
	json_filename = os.path.join(gestures_dir, 'expressions.json')
	
	# Debugging: Print the file path to verify it's correct
	print(f"Looking for expressions.json at: {json_filename}")
	
	if not os.path.exists(json_filename):
		print(f"Error: {json_filename} not found. Running training...")
		capture_expression()  # If the file doesn't exist, run the training function
		return  # After training, exit to allow a fresh run of prediction

	# Load the expression data from the JSON file
	with open(json_filename, 'r') as json_file:
		expressions_data = json.load(json_file)
	
	# Initialize MediaPipe FaceMesh
	mp_face_mesh = mp.solutions.face_mesh
	face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
	mp_drawing = mp.solutions.drawing_utils

	# Define the landmarks that are relevant for expressions (eyes, mouth, eyebrows)
	relevant_landmarks = [
		# Eyes (left and right eyes, around the eye area)
		33, 133, 160, 158, 144, 145, 159, 130, 243, 473, 474, 263,
		# Eyebrows (for furrowed brows, surprise)
		70, 63, 105, 66, 55, 285, 284, 300, 296,
		# Nose (for expressions involving nose movements)
		1, 2, 4, 6, 168, 197,
		# Mouth (for smile/frown)
		61, 185, 317, 402, 81, 375, 291, 56, 185, 85, 312, 97, 91
	]

	# Set up webcam for prediction
	cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

	if not cap.isOpened():
		print("Error: Could not open webcam.")
		return

	# Predict expression based on landmarks
	while True:
		ret, frame = cap.read()
		if not ret:
			print("Error: Failed to capture frame.")
			break
		
		# Convert the frame to RGB for MediaPipe processing
		rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		results = face_mesh.process(rgb_frame)

		# Debug: Check if landmarks are detected
		if results.multi_face_landmarks:
			print("Landmarks detected.")
			face_landmarks = results.multi_face_landmarks[0]
			# Extract only relevant landmarks
			normalized_landmarks = np.array([[lm.x, lm.y, lm.z] for i, lm in enumerate(face_landmarks.landmark) if i in relevant_landmarks])

			# Normalize the prediction landmarks (values between 0 and 1)
			normalized_landmarks = normalized_landmarks / np.max(np.abs(normalized_landmarks), axis=0)  # Normalize by the max absolute value of each axis

			# Estimate the head pose to account for tilting
			yaw, pitch, roll = get_head_pose(face_landmarks.landmark)

			# Normalize or adjust landmarks based on head pose (simplified for now)
			# You can add more sophisticated head pose adjustment logic if necessary

			# Initialize the best match and minimum distance
			best_match = None
			min_distance = float('inf')

			# We process each expression's landmark data
			for expression, expression_samples in expressions_data.items():
				print(f"Processing expression: {expression}")  # Debugging statement
				for stored_landmarks in expression_samples:
					# Extract the relevant landmarks from the stored data
					stored_landmarks = np.array([[lm['x'], lm['y'], lm['z']] for i, lm in enumerate(stored_landmarks) if i in relevant_landmarks])

					# Normalize the stored landmarks to the same scale (values between 0 and 1)
					stored_landmarks = stored_landmarks / np.max(np.abs(stored_landmarks), axis=0)  # Normalize by the max absolute value of each axis

					# Ensure both arrays are the same shape before comparing
					if normalized_landmarks.shape == stored_landmarks.shape:
						distance = np.linalg.norm(normalized_landmarks - stored_landmarks)

						# Log the distance for debugging purposes
						print(f"Distance for expression '{expression}': {distance}")  # Debugging statement

						# Update best match based on minimum distance
						if distance < min_distance:
							min_distance = distance
							best_match = expression

			# Display the predicted expression on the frame
			if best_match:
				cv2.putText(frame, f"Expression: {best_match}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
			else:
				cv2.putText(frame, "Expression: None", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

			# Draw the landmarks (small blue dots) on the predicted face
			for i in relevant_landmarks:  # Loop through the relevant landmarks
				if i < len(face_landmarks.landmark):  # Check if the index is within the range
					landmark = face_landmarks.landmark[i]
					x = int(landmark.x * frame.shape[1])
					y = int(landmark.y * frame.shape[0])
					cv2.circle(frame, (x, y), 1, (255, 0, 0), -1)  # Blue dot with size 1
		else:
			print("No landmarks detected.")  # Debugging statement

		# Show the video feed with the predicted expression
		cv2.imshow('Expression Prediction', frame)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()


# Main flow
if __name__ == "__main__":
	# First, try to run the prediction or training if necessary
	predict_expression()
	
	# Uncomment the following line to run the training if needed
	# capture_expression()
