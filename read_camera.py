import cv2
import time

# Open a connection to the camera (0 is usually the default camera)
cap = cv2.VideoCapture("E:/USER/HKI_2024_2025/ISLab/PROJECT_TDT/test/videos_rasp/video_10.mp4")
if not cap.isOpened():
    print("Error: Could not open camera.")
else:
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        # If frame is read correctly, ret is True
        if not ret:
            print("Error: Failed to grab frame.")
            break

        # Display the frame
        
        cv2.imshow('Camera Feed', frame)
        time.sleep(1/30)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()
