import cv2
import numpy as np
import os

# Load camera calibration data
calibration_file = os.path.dirname(os.path.abspath(__file__)) + "/calibration_data_chessboard.npz"
calib_data = np.load(calibration_file)
mtx = calib_data['mtx']  # Camera matrix
dist = calib_data['dist']  # Distortion coefficients

# Real-world tag size in meters
tag_size = 0.164  # Adjust to the actual size of your AprilTag

# Load the AprilTag dictionary (tag36h11)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# Set up detector parameters
aruco_params = cv2.aruco.DetectorParameters()

# Open video stream (1 for second camera, 0 for default camera)
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit(1)

# Set window size
cv2.namedWindow('AprilTag Distance', cv2.WINDOW_NORMAL)
cv2.resizeWindow('AprilTag Distance', 1920, 1080)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Undistort the frame using the camera matrix and distortion coefficients
    undistorted_frame = cv2.undistort(frame, mtx, dist)

    # Convert frame to grayscale (needed for detection)
    gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        for i, corner in enumerate(corners):
            # Draw the detected markers
            cv2.aruco.drawDetectedMarkers(undistorted_frame, corners, ids)

            # Estimate pose of each marker
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, tag_size, mtx, dist)

            # Draw the axis for the detected marker
            cv2.drawFrameAxes(undistorted_frame, mtx, dist, rvec[0], tvec[0], 0.1)

            # Calculate the distance to the tag
            distance = np.linalg.norm(tvec)

            # Display the distance
            center = tuple(np.mean(corner[0], axis=0).astype(int))
            cv2.putText(undistorted_frame, f"Distance: {distance:.2f} m", center, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    # Display the resulting frame
    cv2.imshow('AprilTag Distance', undistorted_frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
