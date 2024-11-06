import cv2
import numpy as np


def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    hue = hsvC[0][0][0]  # Get the hue value

    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit



class VideoProcessor:
    def __init__(self, path):
        self.path = path
        self.cap = cv2.VideoCapture(path)

        if not self.cap.isOpened():
            print("Error: Could not open video file.")
            exit()

        self.fps = self.cap.get(cv2.CAP_PROP_FPS)

    def process_frame(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get the difference between adjacent pixels in the HSV channels
        diff_h = cv2.absdiff(hsv[:, 1:], hsv[:, :-1])  # width-1
        diff_s = cv2.absdiff(hsv[1:, :], hsv[:-1, :])  # height-1

        # Convert differences to binary using a threshold
        
        thread_hold = 10
        
        _, binary_h = cv2.threshold(diff_h, thread_hold, 255, cv2.THRESH_BINARY)
        _, binary_s = cv2.threshold(diff_s, thread_hold, 255, cv2.THRESH_BINARY)

        # Convert to single-channel by taking only one channel (the first one)
        binary_h_single = binary_h[:, :, 0]  # Take the first channel
        binary_s_single = binary_s[:, :, 0]  # Take the first channel

        # Resize binary images to match the frame size
        binary_h_resized = cv2.resize(binary_h_single, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
        binary_s_resized = cv2.resize(binary_s_single, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Combine binary images
        binary_combined = cv2.bitwise_or(binary_h_resized, binary_s_resized)
        
        kernel = np.ones((8, 8), np.uint8)
        binary_combined = cv2.dilate(binary_combined, kernel, iterations=1)

        # Create masks for red, yellow, green, purple, and blue
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        lower_yellow = np.array([20, 50, 50])
        upper_yellow = np.array([40, 255, 255])
        lower_green = np.array([50, 50, 50])
        upper_green = np.array([70, 255, 255])
        lower_purple = np.array([130, 50, 50])
        upper_purple = np.array([160, 255, 255])
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])

        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_purple = cv2.inRange(hsv, lower_purple, upper_purple)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Apply the masks to the binary combined image
        binary_combined_red = cv2.bitwise_or(binary_combined, binary_combined, mask=mask_red)
        binary_combined_yellow = cv2.bitwise_or(binary_combined, binary_combined, mask=mask_yellow)
        binary_combined_green = cv2.bitwise_or(binary_combined, binary_combined, mask=mask_green)
        binary_combined_purple = cv2.bitwise_or(binary_combined, binary_combined, mask=mask_purple)
        binary_combined_blue = cv2.bitwise_or(binary_combined, binary_combined, mask=mask_blue)

        # Detect circles
        contours, _ = cv2.findContours(binary_combined_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        circles_red = self.detect_circles(binary_combined_red, contours)

        # contours, _ = cv2.findContours(binary_combined_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # circles_yellow = self.detect_circles(binary_combined_yellow, contours)

        # contours, _ = cv2.findContours(binary_combined_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # circles_green = self.detect_circles(binary_combined_green, contours)

        # contours, _ = cv2.findContours(binary_combined_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # circles_purple = self.detect_circles(binary_combined_purple, contours)

        # contours, _ = cv2.findContours(binary_combined_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # circles_blue = self.detect_circles(binary_combined_blue, contours)

        # return binary_combined, binary_combined_red, binary_combined_yellow, binary_combined_green, binary_combined_purple, binary_combined_blue, circles_red, circles_yellow, circles_green, circles_purple, circles_blue

        return binary_combined, binary_combined_red, circles_red
        
    def detect_circles(self, frame, contours):
        circles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            print(area)
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w)/h
            print(aspect_ratio)
            if area > 1:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                circles.append((center, radius))
        return circles

    def run(self):
        while True:
            ret, frame = self.cap.read()

            if not ret:
                print("Error: Failed to grab frame.")
                break

            # binary_combined, binary_combined_red, binary_combined_yellow, binary_combined_green, binary_combined_purple, binary_combined_blue, circles_red, circles_yellow, circles_green, circles_purple, circles_blue = self.process_frame(frame)

            binary_combined, binary_combined_red, circles_red = self.process_frame(frame)
            
            for center, radius in circles_red:
                cv2.circle(frame, center, radius, (0, 0, 255), 2)
                
            # for center, radius in circles_yellow:
            #     cv2.circle(binary_combined_yellow, center, radius, (0, 255, 255), 2)
            # for center, radius in circles_green:
            #     cv2.circle(binary_combined_green, center, radius, (0, 255, 0), 2)
            # for center, radius in circles_purple:
            #     cv2.circle(binary_combined_purple, center, radius, (128, 0, 128), 2)
            # for center, radius in circles_blue:
            #     cv2.circle(binary_combined_blue, center, radius, (255, 0, 0), 2)

            
            cv2.imshow('Camera Feed', frame)
            cv2.imshow('Binary', binary_combined)
            cv2.imshow('Red', binary_combined_red)
            # cv2.imshow('Yellow', binary_combined_yellow)
            # cv2.imshow('Green', binary_combined_green)
            # cv2.imshow('Purple', binary_combined_purple)
            # cv2.imshow('Blue', binary_combined_blue)

            if cv2.waitKey(int(100 / self.fps)) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    processor = VideoProcessor("E:/USER/HKI_2024_2025/ISLab/PROJECT_TDT/test/videos_rasp/video_10.mp4")
    processor.run()


