from dronekit import connect, VehicleMode
import time

# Connect to the Vehicle
vehicle = connect('/dev/ttyS0', baud=115200, wait_ready=True)

# Check if optical flow data is available
def get_local_position():
    if vehicle.location.local_frame:
        print("Local Position (NED):")
        print(f"North: {vehicle.location.local_frame.north}")
        print(f"East: {vehicle.location.local_frame.east}")
        print(f"Down: {vehicle.location.local_frame.down}")
    else:
        print("Waiting for local position data...")

# Main loop
try:
    while True:
        get_local_position()
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    vehicle.close()
