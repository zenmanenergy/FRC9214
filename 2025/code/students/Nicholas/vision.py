import cv2
print("\n"*3)

for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera index {i} is available.")
        cap.release()
    else:
        print(f"Camera index {i} is not available.")