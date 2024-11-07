from control import YawController, NavController
from dronekit import connect, VehicleMode
import time

if __name__ == '__main__':
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    yaw_control = YawController(vehicle)
    yaw_control.set_yaw_speed(20)
    nav_control = NavController(vehicle)
    nav_control.set_velocity_x(1.0)
    
    vehicle.mode = VehicleMode("GUIDED")
    
    vehicle.armed = True
    
    while not vehicle.armed:
        vehicle.armed = True
        time.sleep(.6)
        
    altitude = 1.5
    vehicle.simple_takeoff(altitude)
    
    while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)
      
    while True:
        print(f"heading: {vehicle.heading}, altitude: {vehicle.location.global_relative_frame.alt}")
        
        a = int(input("yaw: "))
        b = int(input("distance: "))
        nav_control.set_distance(b)
        yaw_control.set_yaw(a)
        x = input("set: ")
        
        if x == "a":
            yaw_control.set_flag_control_yaw()
        elif x == "b":
            nav_control.set_flag_control_nav()
        elif x == "c":
            yaw_control.set_flag_control_yaw()
            nav_control.set_flag_control_nav()
        elif x == "d":
            vehicle.mode = VehicleMode("LAND")
        elif x == "q":
            quit()
        
        yaw_control.control_yaw()
        nav_control.control_forward()
    