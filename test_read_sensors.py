import time
from dronekit import connect

# Connect to the drone using serial port /dev/ttyS0 with baud rate 115200
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# Initialize variables to hold raw IMU data
accel_data = {"x": 0, "y": 0, "z": 0}
gyro_data = {"x": 0, "y": 0, "z": 0}
mag_data = {"x": 0, "y": 0, "z": 0}

# MAVLink callback for RAW_IMU data
@vehicle.on_message('RAW_IMU')
def listener_raw_imu(self, name, message):
    global accel_data, gyro_data, mag_data
    accel_data = {"x": message.xacc, "y": message.yacc, "z": message.zacc}
    gyro_data = {"x": message.xgyro, "y": message.ygyro, "z": message.zgyro}
    mag_data = {"x": message.xmag, "y": message.ymag, "z": message.zmag}

# Function to display IMU, gyro, and compass data
def read_sensor_data():
    # IMU attitude data
    print(f"Attitude - Roll: {vehicle.attitude.roll}, Pitch: {vehicle.attitude.pitch}, Yaw: {vehicle.attitude.yaw}")

    # Heading from compass
    print(f"Compass - Heading: {vehicle.heading}")

    # Print raw IMU data (accelerometer, gyroscope, and magnetometer)
    print(f"IMU - Accel: {accel_data}")
    print(f"IMU - Gyro: {gyro_data}")
    print(f"IMU - Mag Field: {mag_data}")

# Loop to keep reading sensor data
try:
    while True:
        read_sensor_data()
        time.sleep(1)  # Adjust the delay as needed
except KeyboardInterrupt:
    print("Interrupted, exiting.")
finally:
    vehicle.close()
