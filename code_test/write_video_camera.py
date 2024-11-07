import cv2
import os
import time

# Create a folder named 'videos' if it doesn't exist
output_folder = 'videos_output'
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Function to generate a unique filename
def get_unique_filename(base_path, base_name, extension):
    counter = 1
    new_filename = f"{base_name}.{extension}"
    while os.path.exists(os.path.join(base_path, new_filename)):
        new_filename = f"{base_name}_{counter}.{extension}"
        counter += 1
    return new_filename

# Open a connection to the camera (0 is usually the default camera)
cap = cv2.VideoCapture(1)

# Define the codec
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4 format

# Generate a unique filename
base_filename = 'video'
unique_filename = get_unique_filename(output_folder, base_filename, 'mp4')
output_file_path = os.path.join(output_folder, unique_filename)

fps = 30.0

# Create VideoWriter object
out = cv2.VideoWriter(output_file_path, fourcc, fps, (640, 640))  # 20 FPS, resolution 640x640

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

        # Resize the frame to 640x640
        frame = cv2.resize(frame, (640, 640))

        # Write the frame to the output file
        out.write(frame)

        # Display the frame
        cv2.imshow('Camera Feed', frame)

        time.sleep(1/fps)
        
        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the camera, VideoWriter, and close windows
cap.release()
out.release()
cv2.destroyAllWindows()

print(f"Video saved as: {unique_filename}")
