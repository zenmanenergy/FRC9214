import cv2
import mediapipe as mp
import numpy as np
import time

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# Training data for each finger count (1-5)
training_data = {}

def get_finger_distances(hand_landmarks):
	"""Calculate distances between key landmarks to represent hand pose."""
	distances = []
	# Use pairs of landmarks to calculate distances
	pairs = [
		(mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.WRIST),
		(mp_hands.HandLandmark.MIDDLE_FINGER_TIP, mp_hands.HandLandmark.WRIST),
		(mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.WRIST),
		(mp_hands.HandLandmark.PINKY_TIP, mp_hands.HandLandmark.WRIST),
		(mp_hands.HandLandmark.THUMB_TIP, mp_hands.HandLandmark.WRIST)
	]
	for tip, base in pairs:
		pt1 = hand_landmarks.landmark[tip]
		pt2 = hand_landmarks.landmark[base]
		dist = np.sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2)
		distances.append(dist)
	return distances

def main():
	cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
	if not cap.isOpened():
		print("Error: Could not open camera.")
		return

	# Training phase for 1 to 5 fingers
	for i in range(1, 6):
		print(f"Please hold up {i} fingers...")
		start_time = time.time()
		while True:
			ret, frame = cap.read()
			if not ret:
				print("Error: Failed to capture frame.")
				break

			frame = cv2.flip(frame, 1)
			rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
			results = hands.process(rgb_frame)
			h, w, _ = frame.shape

			# Display training instructions and countdown
			elapsed_time = int(time.time() - start_time)
			if elapsed_time <= 3:
				cv2.putText(frame, f"Hold up {i} fingers", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
				countdown_text = str(3 - elapsed_time)
				cv2.putText(frame, countdown_text, (w // 2 - 20, h // 2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
			else:
				# Capture landmarks once the countdown is finished
				if results.multi_hand_landmarks:
					for hand_landmarks in results.multi_hand_landmarks:
						mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
						training_data[i] = get_finger_distances(hand_landmarks)
						print(f"Captured training data for {i} fingers.")
					break  # Exit loop after capturing the training data

			cv2.imshow('Training View', frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

	print("Training complete. Starting recognition...")

	# Recognition phase
	while True:
		ret, frame = cap.read()
		if not ret:
			print("Error: Failed to capture frame.")
			break

		frame = cv2.flip(frame, 1)
		rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		results = hands.process(rgb_frame)

		if results.multi_hand_landmarks:
			for hand_landmarks in results.multi_hand_landmarks:
				mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

				# Get the current distances for comparison
				current_distances = get_finger_distances(hand_landmarks)

				# Find the closest matching training data
				closest_match = None
				min_distance = float('inf')
				for finger_count, train_distances in training_data.items():
					distance = np.linalg.norm(np.array(train_distances) - np.array(current_distances))
					if distance < min_distance:
						min_distance = distance
						closest_match = finger_count

				# Display the recognized number of fingers
				cv2.putText(frame, f"Recognized Fingers: {closest_match}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
				print(f"Detected fingers: {closest_match}")

		cv2.imshow('Recognition View', frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()
