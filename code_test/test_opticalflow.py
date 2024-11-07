import time
from dronekit import connect

# Connect to the drone using serial port /dev/ttyS0 with baud rate 115200
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# MTF-01 specific focal length (adjust this based on MTF-01 specifications)
focal_length_mtf01 = 15.6  # Example value; adjust based on actual focal length
x_pos, y_pos = 0, 0  # Initialize position variables

# Function to handle optical flow data and calculate displacement
def optical_flow_callback(self, name, message):
    global x_pos, y_pos
    flow_x = message.flow_x
    flow_y = message.flow_y
    ground_distance = message.ground_distance
    print(f"Flow X: {flow_x}, Flow Y: {flow_y}, Ground Distance: {ground_distance}")
    # Check if ground distance is valid to prevent divide-by-zero errors
    if ground_distance > 0:
        # Calculate displacement using MTF-01 parameters
        delta_x = (flow_x * ground_distance) / focal_length_mtf01
        delta_y = (flow_y * ground_distance) / focal_length_mtf01

        # Integrate displacement to estimate position
        x_pos += delta_x
        y_pos += delta_y

        # Print fused position for monitoring
        print(f"Position X: {x_pos:.2f}, Position Y: {y_pos:.2f}")
    # else:
    #     print("Invalid ground distance, skipping measurement.")

# Register optical flow callback for MTF-01 data
vehicle.add_message_listener('OPTICAL_FLOW', optical_flow_callback)

# Keep script running to continuously process data
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Interrupted, exiting.")
finally:
    vehicle.close()
