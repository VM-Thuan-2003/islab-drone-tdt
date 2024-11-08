import time
import math

from dronekit import connect, VehicleMode # type: ignore
from logs import Printer # type: ignore

class Vehicle:
    def __init__(self, string_connect, ena_logs=True):
        
        self._ena_logs = ena_logs
        self._print = Printer(ena_logs=self._ena_logs, name="Vehicle")
        
        self._string_connect = string_connect
        self._vehicle = connect(self._string_connect,baud= 57600, wait_ready=True)

        self.mid_throttle = 0.5
        self.max_throttle = 0.9
        self.min_throttle = 0.3
        self.thrust_factor = 0.1

        self._print.print_success("Vehicle Connected")
        
    def close(self):
        self._print.print_success("Vehicle Disconnected")
        self._vehicle.close()
    
    def change_mode(self, mode: str):
        self._vehicle.mode = VehicleMode(mode)
    
    def arm(self):
        self._vehicle.armed = True
        while not self._vehicle.armed:
            self._vehicle.armed = True
            time.sleep(.6)
    
    def take_off(self, aTargetAltitude):
        
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6
        
        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = self._vehicle.location.global_relative_frame.alt
            print(" Altitude: %f  Desired: %f" %
                (current_altitude, aTargetAltitude))
            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                print("Reached target altitude")
                break
            elif current_altitude >= aTargetAltitude*0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            self.set_attitude(thrust = thrust)
            time.sleep(0.2)
        
    def to_quaternion(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    def send_attitude_target(self, roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
        """
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                    When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw_angle = self._vehicle.attitude.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self._vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian/second
            thrust  # Thrust
        )
        self._vehicle.send_mavlink(msg)
    
    def set_attitude(self, roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """
        self.send_attitude_target(roll_angle, pitch_angle,
                            yaw_angle, yaw_rate, False,
                            thrust)
        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                yaw_angle, yaw_rate, False,
                                thrust)
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0,
                            0, 0, True,
                            thrust)
        
        