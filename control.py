from math import sqrt
from pymavlink import mavutil  # Import mavutil for MAVLink commands
import time


class YawController:
    def __init__(self, vehicle):
        """
        Initialize the YawController with a vehicle.
        
        Parameters:
            vehicle (Vehicle): vehicle object to control the yaw of
        """
        
        self.vehicle = vehicle
        
        self.relative = False # True to interpret the heading as relative to the current yaw, False for absolute yaw
        self.direction = 1 # 1 for clockwise, -1 for counter-clockwise
        self.yaw_speed = 10.0 # Speed of yaw rotation in degrees per second
        
        self.yaw = 0.0 # Target yaw angle
        
        self.flag_control_yaw = False # Flag to control yaw
        
    def set_relative(self, relative = False):
        """
        Set whether the target yaw is relative to the current yaw or absolute

        Parameters:
            relative (bool): True to interpret the heading as relative to the current yaw, False for absolute yaw
        """
        self.relative = relative
    
    def set_yaw(self, yaw):
        """
        Set the target yaw angle in degrees

        Parameters:
            yaw (float): Target yaw angle in degrees. If yaw is between -180 and 180, the shortest path will be taken to the target yaw.
                         If yaw is outside of this range, the direction will be determined by the sign of the yaw value.
        """
        self.yaw = yaw
        
        rel_yaw = self.vehicle.heading
        print(f"rel_yaw: {rel_yaw}, yaw: {self.yaw}, rel_yaw - yaw: {rel_yaw - self.yaw}")
        if rel_yaw - self.yaw > 0:
            self.direction = -1
        else:
            self.direction = 1
        
    def set_yaw_speed(self, yaw_speed = 10):
        """
        Set the yaw speed in degrees per second

        Parameters:
            yaw_speed (float): Yaw speed in degrees per second
        """
        self.yaw_speed = yaw_speed
    
    def set_flag_control_yaw(self):
        
        """
        Set the flag to control yaw
        
        This function sets the flag to control yaw, which makes the control_yaw function
        send MAV_CMD_CONDITION_YAW command to point the vehicle to a specified heading
        (in degrees). The target yaw angle is set by the set_yaw function, and the yaw
        speed is set by the set_yaw_speed function.
        """
        self.flag_control_yaw = True
    
    def control_yaw(self):
        
        if self.flag_control_yaw == True:
            
            self.condition_yaw(self.yaw)

            rel_yaw = self.vehicle.heading
            
            compare_yaw = abs(self.yaw - rel_yaw)
            
            time.sleep(float(compare_yaw / self.yaw_speed))
            
            self.flag_control_yaw = False
    
    def condition_yaw(self, heading):
        """
        Send MAV_CMD_CONDITION_YAW command to point the vehicle to a specified heading (in degrees).
        
        Parameters:
        heading (float): Target heading in degrees.
        direction (int): 1 for clockwise, -1 for counter-clockwise.
        relative (bool): True to interpret the heading as relative to the current yaw.
        yaw_speed (float): Speed of yaw rotation in degrees per second.
        """
        
        # Create the MAV_CMD_CONDITION_YAW command
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,       # confirmation
            heading, # param 1: target heading in degrees
            self.yaw_speed,  # param 2: yaw speed in deg/s
            self.direction,  # param 3: direction -1 ccw, +1 cw
            self.relative,  # param 4: relative/absolute
            0, 0, 0)  # param 5-7 (not used)
        
        # Send the command to the vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()


class NavController:
    def __init__(self, vehicle):
        self.vehicle = vehicle
    
        self.flag_control_nav = False

        self.velocity_x = 1.0
        self.distance = 1.0
    
        self.off_set = .2
    
    def set_velocity_x(self, velocity_x=1.0):
        """
        Set the forward velocity for navigation.

        Parameters:
        velocity_x (float): The forward velocity in meters per second.
        """
        self.velocity_x = velocity_x
    
    def set_distance(self, distance=1.0):
        """
        Set the distance for navigation.

        Parameters:
        distance (float): The distance in meters that the vehicle should move.
        """
        self.distance = distance
    
    def set_flag_control_nav(self):
        """
        Set the flag to control navigation
        
        This function sets the flag to control navigation, which makes the control_nav function
        send the vehicle to the specified location.
        """
        self.flag_control_nav = True
    
    def control_forward(self):
        
        if self.flag_control_nav == True:
            
            self.send_forward(self.velocity_x, self.distance)
            
            self.flag_control_nav = False
    
    def send_forward(self, velocity_x, distance):
        
        """
        Move vehicle in specified direction without GPS.

        Parameters:
        velocity_x (float): Forward velocity component (m/s)
        distance (float): Distance to move in meters

        Returns:
        None
        """
        
        duration = distance / velocity_x
        
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  # Only velocity enabled
            0, 0, 0,
            velocity_x, 0, 0,
            0, 0, 0,
            0, 0
        )
        self.vehicle.send_mavlink(msg)
        time.sleep(duration - self.off_set)
