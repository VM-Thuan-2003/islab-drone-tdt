from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# Connect to the Vehicle
try:
    vehicle = connect('udp:127.0.0.1:14550', baud=115200, wait_ready=True)
except Exception as e:
    print(f"Error connecting to vehicle: {e}")
    exit()

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in specified direction without GPS.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # Only velocity enabled
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0
    )
    for _ in range(int(duration * 10)):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

# Example usage
try:
        print("Moving forward")
        send_ned_velocity(1, 0, 0, 1)  # Move forward at 1 m/s for 5 seconds
finally:
    print("Landing")
    # vehicle.mode = VehicleMode("LAND")
    # while vehicle.armed:
    #     time.sleep(1)
    vehicle.close()
