from dronekit import connect, VehicleMode
from pymavlink import mavutil  # Import mavutil for MAVLink commands
import time

# Connect to the Vehicle
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

def condition_yaw(heading, direction=1, relative=False, yaw_speed=10):
    """
    Send MAV_CMD_CONDITION_YAW command to point the vehicle to a specified heading (in degrees).
    
    Parameters:
    heading (float): Target heading in degrees.
    direction (int): 1 for clockwise, -1 for counter-clockwise.
    relative (bool): True to interpret the heading as relative to the current yaw.
    yaw_speed (float): Speed of yaw rotation in degrees per second.
    """
    if relative:
        is_relative = 1  # Yaw relative to current heading
    else:
        is_relative = 0  # Yaw is an absolute angle

    # Create the MAV_CMD_CONDITION_YAW command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,       # confirmation
        heading, # param 1: target heading in degrees
        yaw_speed,  # param 2: yaw speed in deg/s
        direction,  # param 3: direction -1 ccw, +1 cw
        is_relative,  # param 4: relative/absolute
        0, 0, 0)  # param 5-7 (not used)
    
    # Send the command to the vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def control_yaw(heading):
    global vehicle
    
    rel_heading = vehicle.heading

    direction = 1

    
    if heading > rel_heading:
        direction = 1
    elif heading < rel_heading:
        direction = -1
    
    # condition_yaw(heading, direction=direction, relative=True, yaw_speed=10)
    
    print(heading - rel_heading)

    time.sleep(1)

# Example usage
try:
    # Set the drone mode to GUIDED
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == "GUIDED":
        print("Waiting for drone to enter GUIDED mode")
        time.sleep(1)

    # Set target yaw angle
    target_yaw = 90  # Target yaw in degrees
    condition_yaw(target_yaw, direction=1, relative=False, yaw_speed=10)
    time.sleep(1)  # Give it time to complete yaw
    
    while True:
        target_yaw = float(input("input target yaw: "))
        condition_yaw(target_yaw, direction=1, relative=False, yaw_speed=10)
        time.sleep(1)
    
finally:
    # Close the vehicle connection
    vehicle.close()
