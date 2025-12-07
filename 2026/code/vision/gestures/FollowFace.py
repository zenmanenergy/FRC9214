import cv2
import mediapipe as mp

# Initialize mediapipe face mesh modules
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils

# Set up the webcam
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Initialize the FaceMesh model
with mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5) as face_mesh:
	
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

		# If face landmarks are found, calculate yaw and pitch and draw landmarks
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

		# Display the result
		cv2.imshow('Face Detection with Yaw, Pitch, and Facial Features', frame)

		# Exit on pressing 'q'
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
