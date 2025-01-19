import os
import cv2
import glob
import numpy as np

# Get the current script directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Define the folder path containing the chessboard images
folder_path = os.path.join(current_dir, 'images', '*.png')

# Chessboard dimensions (inner corners per row and column)
chessboard_size = (9, 6)

square_size = 0.024  # size of each square on chessboard in meters

# Prepare object points (3D points in real-world space)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

objp *= square_size

# Arrays to store object points and image points
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane
img_shape = None

# Use glob to find all matching image files
image_files = glob.glob(folder_path)
if not image_files:
    print(f"No images found in folder: {os.path.join(current_dir, 'images')}")
    exit(1)

# Loop through the images
for file_path in image_files:
    img = cv2.imread(file_path)  # Load the image
    if img is None:
        print(f"Failed to load image: {file_path}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
    
    # Set image shape based on the first image
    if img_shape is None:
        img_shape = gray.shape[::-1]  # Store image dimensions as (width, height)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        objpoints.append(objp)  # Add object points
        imgpoints.append(corners)  # Add detected corner points

        # Draw the corners on the image for visualization
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
    else:
        print(f"Chessboard not detected in image: {os.path.basename(file_path)}")

cv2.destroyAllWindows()

# Calibrate the camera using the collected points
if len(objpoints) > 0 and len(imgpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)
    if not ret:
        print("Calibration failed.")
        exit(1)

    print(f"Camera Matrix:\n{mtx}")
    print(f"Distortion Coefficients:\n{dist}")

    # Use the original image size
    w, h = img_shape[1], img_shape[0]
    
    # Get the optimal new camera matrix (for undistortion)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # Apply undistortion to the original image
    img_undistorted = cv2.undistort(img, mtx, dist, None, newcameramtx)
    
    # Show the undistorted image
    cv2.imshow("Undistorted Image", img_undistorted)
    cv2.waitKey(5000)  # Wait 5 seconds for display

    print("Camera calibrated successfully.")

    # Save calibration data
    np.savez(os.path.join(current_dir, 'calibration_data_chessboard.npz'), mtx=mtx, dist=dist)
    print(f"Calibration data saved to: {os.path.abspath('calibration_data_chessboard.npz')}")
else:
    print("Not enough data to perform calibration.")
