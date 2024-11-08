import cv2
import time

# List of available backends to test
backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_V4L2, cv2.CAP_FFMPEG, cv2.CAP_GSTREAMER]
backend_names = ["DirectShow", "MSMF", "V4L2", "FFmpeg", "GStreamer"]

# Function to measure time to open camera
def test_camera_initialization(backend, backend_name):
	start_time = time.time()  # Start timer
	
	cap = cv2.VideoCapture(1, backend)  # Try to open the camera with the specific backend
	if cap.isOpened():
		elapsed_time = time.time() - start_time  # End timer
		print(f"{backend_name} opened the camera in {elapsed_time:.4f} seconds.")
		cap.release()
	else:
		print(f"{backend_name} failed to open the camera.")

if __name__ == "__main__":
	for backend, backend_name in zip(backends, backend_names):
		print(f"Testing {backend_name} backend...")
		test_camera_initialization(backend, backend_name)
		print("-" * 40)  # Separator between tests
