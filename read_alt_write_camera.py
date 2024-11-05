from dronekit import connect, VehicleMode
import time
import cv2
from datetime import datetime
import os  # Import os module to handle directory creation
import threading  # Import threading module

# Connect to the Vehicle
vehicle = connect('com16', wait_ready=True, baud=115200)

# Set initial home altitude
home_alt = vehicle.location.global_frame.alt

def get_relative_altitude():
    """
    Calculates an alternative to global_relative_frame.alt using home altitude.
    """
    print("Current altitude:", vehicle.location.global_frame.alt)
    current_alt = vehicle.location.global_frame.alt
    relative_altitude = current_alt - home_alt
    return relative_altitude

# Video writing thread function
def video_capture(video_filename, stop_event):
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Could not open camera.")
        return

    # Initialize video writer
    video_out = cv2.VideoWriter(video_filename, cv2.VideoWriter_fourcc(*'mp4v'), 10, (640, 480))

    while not stop_event.is_set():
        ret, frame = camera.read()
        if ret:
            video_out.write(frame)

            # Display the frame (optional)
            cv2.imshow("Camera Feed", frame)
            
            # Exit on 'q' key press (check after a small delay)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break
        else:
            print("Error: Could not read frame.")
        
        time.sleep(0.1)  # Small delay to control the frame rate

    # Release resources
    camera.release()
    video_out.release()
    cv2.destroyAllWindows()

# Main function to log altitude data
def log_altitude_data(text_filename, stop_event):
    with open(text_filename, "w") as file:
        while not stop_event.is_set():
            rel_alt = get_relative_altitude()
            print("Relative Altitude:", rel_alt)
            file.write(f"Relative Altitude: {rel_alt}\n")
            file.flush()
            time.sleep(1)  # Log data every second

# Generate a unique filename based on the current timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

# Specify the directory for output files
output_dir = "folder_alt_video"
os.makedirs(output_dir, exist_ok=True)  # Create directory if it doesn't exist

video_filename = f"{output_dir}/video_output_{timestamp}.mp4"
text_filename = f"{output_dir}/altitude_data_{timestamp}.txt"

# Create a stop event for threads
stop_event = threading.Event()

# Start threads for video capture and altitude logging
video_thread = threading.Thread(target=video_capture, args=(video_filename, stop_event))
altitude_thread = threading.Thread(target=log_altitude_data, args=(text_filename, stop_event))

video_thread.start()
altitude_thread.start()

try:
    # Wait for both threads to finish
    video_thread.join()
    altitude_thread.join()
except KeyboardInterrupt:
    print("Exiting loop")
    stop_event.set()  # Signal threads to stop

# Close the vehicle connection
vehicle.close()
