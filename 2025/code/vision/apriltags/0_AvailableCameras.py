import cv2


print("")
print("")
print("")
# Loop through camera indices to check which are available
for i in range(10):  # Try checking first 10 indices
	cap = cv2.VideoCapture(i)
	if cap.isOpened():
		print(f"Camera index {i} is available.")
		cap.release()
	else:
		print(f"Camera index {i} is not available.")
