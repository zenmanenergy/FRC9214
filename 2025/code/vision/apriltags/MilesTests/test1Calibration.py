import cv2
import numpy as np
from cv2 import aruco

# Create Aruco dictionary and board
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
board = aruco.CharucoBoard_create(squaresX=7, squaresY=5, squareLength=0.04, markerLength=0.02, dictionary=aruco_dict)

# Prepare storage for object and image points
all_corners = []
all_ids = []
img_shape = None

# Load calibration images
images = [cv2.imread(f'image_{i}.jpg') for i in range(10)]

for img in images:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_shape = gray.shape[::-1]

    # Detect markers
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    if len(corners) > 0:
        # Refine with Charuco corners
        _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board)
        if charuco_corners is not None and charuco_ids is not None:
            all_corners.append(charuco_corners)
            all_ids.append(charuco_ids)

# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(all_corners, all_ids, board, img_shape, None, None)

# Save calibration data
np.savez("calibration_data.npz", mtx=mtx, dist=dist)
