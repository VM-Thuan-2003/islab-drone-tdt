from navigation import Navigation
from vehicle import Vehicle
from delay_manager import DelayManager
from thread_manager import ThreadManager
from logs import Printer
import time
class Main:
    def __init__(self, string_connect="udp:127.0.0.1:14550"):
        self._print = Printer(ena_logs=True, name="Main")
        self.vehicle = Vehicle(string_connect, ena_logs=True)
        self.navigation = Navigation(self.vehicle._vehicle, ena_logs=True)
        self.delay_manager = DelayManager()
        # self._thread_manager = ThreadManager(5)
    def run(self):
        try:
            self.vehicle.change_mode("GUIDED")
            self.vehicle.arm()
            
            self.vehicle.take_off(2)
            
            self.vehicle.set_attitude(yaw_angle=0,duration=3)
            
            print("Move forward")
            self.vehicle.set_attitude(yaw_angle=0,pitch_angle = -5, thrust = 0.51, duration = 3)
            time.sleep(5)

            self.vehicle.set_attitude(yaw_angle=180, duration=3)
            time.sleep(2)
            
            print("Move forward")
            self.vehicle.set_attitude(yaw_angle=180, pitch_angle = -5, thrust = 0.51, duration = 3)
            time.sleep(5)
            
            self.vehicle.change_mode("LAND")
            
        except KeyboardInterrupt:
            self._print.print_warning("Keyboard Interrupt")
        except Exception as e:
            self._print.print_error(e)
        finally:
            self._print.print_normal("Finished")
            self.stop()
    
    def stop(self):
        self.vehicle.close()
    
if __name__ == '__main__':
    main = Main()
    main.run()