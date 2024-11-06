import cv2
import numpy as np
import time

from thread_manager import ThreadManager

class ImageProcessing:
    def __init__(self):
        pass
    
        self.input_altitude = 4

        self.max_height = 10
        
        self.value_offset_area = (0,200)
        self.value_offset_wh = (3000,0)
        
        self.offset_camera = 0.3 # 0.3m
        
    def getHSV(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return hsv
    
    def convert_diff_binary(self, hsv, frame, thread_hold=10):
        diff_h = cv2.absdiff(hsv[:, 1:], hsv[:, :-1])  
        diff_s = cv2.absdiff(hsv[1:, :], hsv[:-1, :])  
        
        _, binary_h = cv2.threshold(diff_h, thread_hold, 255, cv2.THRESH_BINARY)
        _, binary_s = cv2.threshold(diff_s, thread_hold, 255, cv2.THRESH_BINARY)
        
        binary_h_single = binary_h[:, :, 0]
        binary_s_single = binary_s[:, :, 0]
        
        binary_h_resized = cv2.resize(binary_h_single, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
        binary_s_resized = cv2.resize(binary_s_single, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
        
        binary_combined_diff = cv2.bitwise_or(binary_h_resized, binary_s_resized)
        
        return binary_combined_diff

    def rgb_to_hsv(self, r, g, b):
        r, g, b = r / 255.0, g / 255.0, b / 255.0
        cmax = max(r, g, b)
        cmin = min(r, g, b)
        delta = cmax - cmin

        if delta == 0:
            h = 0
        elif cmax == r:
            h = (60 * ((g - b) / delta) + 360) % 360
        elif cmax == g:
            h = (60 * ((b - r) / delta) + 120) % 360
        elif cmax == b:
            h = (60 * ((r - g) / delta) + 240) % 360

        s = 0 if cmax == 0 else (delta / cmax)
        v = cmax

        return h, s, v
    
    def create_color_mask(self, image, rgb_color, offset=10, saturation = [50, 255], value = [50, 255]):
        hsv_color = self.rgb_to_hsv(rgb_color[0], rgb_color[1], rgb_color[2])
        
        lower_saturation, upper_saturation = saturation
        lower_value, upper_value = value
        
        lower = np.array([max(0, hsv_color[0] - offset), lower_saturation, lower_value], dtype=np.uint8)
        upper = np.array([min(255, hsv_color[0] + offset), upper_saturation, upper_value], dtype=np.uint8)
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, lower, upper)
        
        return mask

    def convert_binary_combined_mask(self, binary_combined_diff, mask):
        binary_combined = cv2.bitwise_and(binary_combined_diff, mask)
        return binary_combined
    
    def dilation(self, image, kernel=(8,8), iterations = 1):
        kernel = np.ones(kernel, np.uint8)
        image_dilation = cv2.dilate(image, kernel, iterations=iterations)
        return image_dilation
    
    def calculate_value_height(self, altitude, height = (0,6), value = (0,200)):
        
        altitude1, value1 = height[0], value[0]
        altitude2, value2 = height[1], value[1]

        m = (value2 - value1) / (altitude2 - altitude1)
        
        c = value1 - m * altitude1
        
        value = m * altitude + c
        
        return value
    
    def calculate_scale_height(self, altitude):
        scale = float(self.offset_camera / altitude)
        return scale
    
    def detect_circle(self, image, altitude = 4):
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        circles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            area_target = self.calculate_value_height(altitude=altitude,height=(0,self.max_height),value=self.value_offset_area)
            wh_target = self.calculate_value_height(altitude=altitude,height=(0,self.max_height),value=self.value_offset_wh)
            
            print(area, area_target, w*h, wh_target)
            
            # print(f"altitude: {altitude} - area: {area} - area_target: {area_target} - w*h: {w*h} - wh_target: {wh_target}")
        
            if area > area_target and w*h > wh_target:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                circles.append((center, radius))
        return circles
    
    def get_input_test(self):
        try:
            while True:
                self.input_altitude = float(input("Enter altitude: "))
        except ValueError:
            print("Invalid input. Please enter a number.")
    
if __name__ == "__main__":
    path = "E:/USER/HKI_2024_2025/ISLab/PROJECT_TDT/test/videos_rasp/video_10.mp4"
    cam = cv2.VideoCapture(path)
    
    if not cam.isOpened():
        print("Error: Could not open video file.")
        exit()
    
    fps = cam.get(cv2.CAP_PROP_FPS)
    print("Frames per second:", fps)
    
    image_processing = ImageProcessing()
    
    cusThread = ThreadManager(1)
    cusThread.start_threads()
    cusThread.add_task(image_processing.get_input_test)
    
    while True:
        ret, frame = cam.read()
        
        if not ret:
            continue
        
        hsv = image_processing.getHSV(frame)
        mask = image_processing.create_color_mask(frame, (100, 0, 0), 10)
        mask_dilation = image_processing.dilation(mask,(7,7),1)
        binary_combined_diff = image_processing.convert_diff_binary(hsv, frame)
        binary_combined = image_processing.convert_binary_combined_mask(binary_combined_diff, mask_dilation)
        
        circle = image_processing.detect_circle(binary_combined, altitude=image_processing.input_altitude)  
        
        for center, radius in circle:
                cv2.circle(frame, center, radius, (0, 0, 255), 2)
        
        cv2.imshow("Camera Feed", frame)
        cv2.imshow("Binary", binary_combined)
        
        time.sleep(1/fps)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cam.release()
    cv2.destroyAllWindows()
    cusThread.stop_threads()

