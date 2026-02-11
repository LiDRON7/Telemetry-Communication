from pymavlink import mavutil
from wakeup import wakeup
import time

the_connection= wakeup()

# Keep reading the mavlink messages. i.e attitude and scaled imu
while True:
    attitude = the_connection.recv_match(type='ATTITUDE', blocking= False) # 30
    if attitude:
        print(attitude)
    scaled_imu = the_connection.recv_match(type="SCALED_IMU", blocking= False) # 26
    if scaled_imu:
        print(scaled_imu)

    time.sleep(0.01)
