import cv2
import mediapipe as mp
import numpy as np
import time

# Initialize MediaPipe
mp_hands = mp.solutions.hands
mp_face = mp.solutions.face_mesh
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
face_mesh = mp_face.FaceMesh(static_image_mode=False, max_num_faces=1, min_detection_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# Dictionary to store training data for each gesture
training_data = {}

# List of gestures to train
gestures = ["holding glasses", "picking nose", "scratching forehead", "rubbing chin"]

def get_landmark_distances(hand_landmarks, face_landmarks):
	"""Calculate distances between key hand and face landmarks to represent gesture."""
	if not hand_landmarks or not face_landmarks:
		return None

	# Define pairs of landmarks for distance calculation
	hand_points = [
		(mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.THUMB_TIP),
		(mp_hands.HandLandmark.MIDDLE_FINGER_TIP, mp_hands.HandLandmark.THUMB_TIP)
	]

	# Face landmarks that are more commonly available (adjust as needed for gestures)
	face_points = [
		(10, 152),  # Between forehead and chin for reference
		(234, 454)  # Cheeks for horizontal reference
	]

	distances = []

	# Calculate hand distances
	for (point1, point2) in hand_points:
		if point1 < len(hand_landmarks.landmark) and point2 < len(hand_landmarks.landmark):
			pt1 = hand_landmarks.landmark[point1]
			pt2 = hand_landmarks.landmark[point2]
			hand_dist = np.sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2)
			distances.append(hand_dist)

	# Calculate distances between face and hand points
	for (face_index, hand_index) in face_points:
		if face_index < len(face_landmarks.landmark) and hand_index < len(hand_landmarks.landmark):
			face_pt = face_landmarks.landmark[face_index]
			hand_pt = hand_landmarks.landmark[hand_index]
			face_hand_dist = np.sqrt((face_pt.x - hand_pt.x)**2 + (face_pt.y - hand_pt.y)**2)
			distances.append(face_hand_dist)

	return distances

def main():
	cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
	if not cap.isOpened():
		print("Error: Could not open camera.")
		return

	# Set a flag to track the training phase
	training_phase = True

	# Training phase for each gesture
	for gesture in gestures:
		print(f"Please perform the gesture: {gesture}")
		start_time = time.time()
		while True:
			ret, frame = cap.read()
			if not ret:
				print("Error: Failed to capture frame.")
				break

			frame = cv2.flip(frame, 1)
			rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
			h, w, _ = frame.shape

			# Process frame with MediaPipe
			hand_results = hands.process(rgb_frame)
			face_results = face_mesh.process(rgb_frame)

			# Display training instructions and countdown
			elapsed_time = int(time.time() - start_time)
			if elapsed_time <= 3:
				cv2.putText(frame, f"Gesture: {gesture}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
				countdown_text = str(3 - elapsed_time)
				cv2.putText(frame, countdown_text, (w // 2 - 20, h // 2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
			else:
				# Capture landmarks once countdown finishes
				if hand_results.multi_hand_landmarks and face_results.multi_face_landmarks:
					for hand_landmarks in hand_results.multi_hand_landmarks:
						for face_landmarks in face_results.multi_face_landmarks:
							# Draw both hand and face landmarks during training
							mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
							mp_drawing.draw_landmarks(frame, face_landmarks, mp_face.FACEMESH_TESSELATION)
							training_data[gesture] = get_landmark_distances(hand_landmarks, face_landmarks)
							print(f"Captured training data for gesture: {gesture}")
					break  # Exit loop after capturing the training data

			cv2.imshow('Training View', frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

	# End of training phase
	training_phase = False
	print("Training complete. Starting recognition...")

	# Recognition phase
	while True:
		ret, frame = cap.read()
		if not ret:
			print("Error: Failed to capture frame.")
			break

		frame = cv2.flip(frame, 1)
		rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		hand_results = hands.process(rgb_frame)
		face_results = face_mesh.process(rgb_frame)

		if hand_results.multi_hand_landmarks and face_results.multi_face_landmarks:
			for hand_landmarks in hand_results.multi_hand_landmarks:
				for face_landmarks in face_results.multi_face_landmarks:
					# Only draw hand landmarks during recognition
					mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
					if training_phase:
						mp_drawing.draw_landmarks(frame, face_landmarks, mp_face.FACEMESH_TESSELATION)

					# Get the current distances for comparison
					current_distances = get_landmark_distances(hand_landmarks, face_landmarks)
					if current_distances is None:
						continue

					# Find the closest matching gesture based on distance similarity
					closest_gesture = None
					min_distance = float('inf')
					for gesture, train_distances in training_data.items():
						distance = np.linalg.norm(np.array(train_distances) - np.array(current_distances))
						if distance < min_distance:
							min_distance = distance
							closest_gesture = gesture

					# Display the recognized gesture
					if closest_gesture:
						cv2.putText(frame, f"Gesture: {closest_gesture}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
						print(f"Recognized gesture: {closest_gesture}")

		cv2.imshow('Recognition View', frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()
