from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

# Connect to the vehicle
vehicle = connect('udp:127.0.0.1:14550', baud=115200, wait_ready=True)

def arm_and_takeoff(target_altitude):
    """ Arms the drone and takes off to a specified altitude """
    print("Arming motors")
    # vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {target_altitude} meters")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height
    while True:
        print("Altitude:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def move_to_position(north, east, down):
    """
    Moves the drone to a specified local position (NED coordinates).
    `north`, `east`, and `down` are in meters.
    """
    # Calculate the desired position
    target_distance = math.sqrt(north**2 + east**2 + down**2)
    
    # Send the position command
    vehicle.simple_goto(LocationGlobalRelative(north, east, down))
    
    # Monitor movement until reaching the target
    while True:
        current_north = vehicle.location.local_frame.north
        current_east = vehicle.location.local_frame.east
        current_down = vehicle.location.local_frame.down

        # Calculate remaining distance to the target
        remaining_distance = math.sqrt(
            (current_north - north)**2 +
            (current_east - east)**2 +
            (current_down - down)**2
        )

        print(f"Distance to target: {remaining_distance:.2f} meters")

        if remaining_distance <= 0.5:  # within 0.5 meter tolerance
            print("Target position reached")
            break
        time.sleep(1)

def land():
    """ Lands the drone """
    print("Landing")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Waiting for landing...")
        time.sleep(1)
    print("Landed and disarmed")


"""
    This script demonstrates basic control of the drone using NED coordinates.

    The drone takes off to a target altitude, then moves to a specified position
    (north, east, down) relative to the starting position.

    It then lands.

    The diagram above shows the relative orientation of the drone
    movements. The drone moves in the direction of the arrow.

    The drone is controlled using the `simple_goto` function, which sends a
    command to move to a specified local position (NED coordinates).

    The `simple_goto` function is a blocking call, so the script waits until
    the drone reaches the target position before continuing.

    The `simple_takeoff` and `land` functions are also blocking, so the script
    waits until the drone takes off or lands before continuing.

         N
         -
   W --------- E
         -
         S
"""

try:
    
    print(vehicle.location.local_frame)
    
    while True:
        
        print(f"Mode: {vehicle.mode.name}")
        
        if vehicle.mode.name == "GUIDED":
        
            # Takeoff to 2 meters
            arm_and_takeoff(2)
            
            # Move to a local position: 1 meters north, 1 meters east, and hold altitude
            print("Moving to position: North 1m, East 1m")
            move_to_position(1, 1, 0)

            # Move to another position: North 1m, East 0m, and hold altitude
            print("Moving to position: North 1m, East 0m")
            move_to_position(1, 0, 0)

            # Move back to the starting position
            print("Returning to start position")
            move_to_position(0, 0, 0)
            
            break


except KeyboardInterrupt:
    # Land the drone
    land()
    print("Exiting...")

finally:
    # land()
    # Close vehicle connection
    vehicle.close()
