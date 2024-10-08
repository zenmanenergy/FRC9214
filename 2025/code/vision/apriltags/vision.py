import cv2
import apriltag

# Initialize camera (use 0 for the default camera or the index for other cameras)
cap = cv2.VideoCapture(0)

# Initialize the AprilTag detector
detector = apriltag.Detector()

# Loop to continuously get frames from the camera
while True:
	# Capture frame-by-frame
	ret, frame = cap.read()
	
	if not ret:
		print("Failed to grab frame")
		break

	# Convert the frame to grayscale (AprilTag detection requires grayscale)
	gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Detect AprilTags in the frame
	tags = detector.detect(gray_frame)

	# Draw a box around each detected tag
	for tag in tags:
		# Extract the corners of the detected tag
		corners = tag.corners
		
		# Convert corners to integer coordinates
		corners = [(int(pt[0]), int(pt[1])) for pt in corners]
		
		# Draw a polygon (box) around the AprilTag
		cv2.polylines(frame, [np.array(corners)], True, (0, 255, 0), 2)
		
		# Optionally, put the tag's ID on the image
		center = (int(tag.center[0]), int(tag.center[1]))
		cv2.putText(frame, str(tag.tag_id), center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

	# Display the resulting frame with detected tags
	cv2.imshow('AprilTag Detection', frame)

	# Break the loop on 'q' key press
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release the camera and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()
