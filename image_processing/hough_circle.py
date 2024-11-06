import cv2
import numpy as np

# Open a connection to the camera (0 is usually the default camera)
path = "E:/USER/HKI_2024_2025/ISLab/PROJECT_TDT/test/videos_rasp/video_8.mp4"
cap = cv2.VideoCapture(path)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to grab frame.")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise and improve edge detection
    gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Use HoughCircles to detect circles
    circles = cv2.HoughCircles(
        gray_blurred,
        cv2.HOUGH_GRADIENT,  # detection method
        dp=1,                # inverse ratio of the accumulator resolution to the image resolution
        minDist=50,          # minimum distance between detected centers
        param1=50,           # higher threshold for the Canny edge detector
        param2=30,           # accumulator threshold for circle centers (lower = more circles detected)
        minRadius=20,        # minimum radius of circles to detect
        maxRadius=100        # maximum radius of circles to detect
    )

    # If some circles are detected, print and draw them
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            center = (circle[0], circle[1])  # circle center
            radius = circle[2]               # circle radius

            # Draw the circle center
            cv2.circle(frame, center, 1, (0, 100, 100), 3)
            # Draw the circle outline
            cv2.circle(frame, center, radius, (255, 0, 255), 3)

    # Display the output
    cv2.imshow("Circles Detected", frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()

