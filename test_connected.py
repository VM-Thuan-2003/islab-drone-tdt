from navigation import Navigation
from vehicle import Vehicle
from delay_manager import DelayManager
from thread_manager import ThreadManager
from logs import Printer
import time
class Main:
    def __init__(self, string_connect="/dev/ttyACM0"):
        self._print = Printer(ena_logs=True, name="Main")
        self.vehicle = Vehicle(string_connect, ena_logs=True)
        self.navigation = Navigation(self.vehicle._vehicle, ena_logs=True)
        self.delay_manager = DelayManager()
        # self._thread_manager = ThreadManager(5)
    def run(self):
        try:
            self.vehicle.change_mode("GUIDED")
            
            print(f"battery percentage: {self.vehicle._vehicle.battery}")     
            
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