"""
Example of how to read all the parameters from the Autopilot with pymavlink
"""

# Disable "Broad exception" warning
# pylint: disable=W0703

import time
import sys
import math

# Import mavutil
from pymavlink import mavutil


# Create the connection
#* edit the port to the one you are using
#master = mavutil.mavlink_connection('/dev/ttyACM0')
master = mavutil.mavlink_connection('udpin:localhost:14551')
# Wait a heartbeat before sending commands
master.wait_heartbeat()


# Request attitude
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 0, 1)

# Wait for attitude data
while True:
    msgIMU = master.recv_match(type='SCALED_IMU2', blocking=True)
    msgAttitude = master.recv_match(type='ATTITUDE', blocking=True)
    if msgAttitude is not None:
        roll_deg = math.degrees(msgAttitude.roll)
        pitch_deg = math.degrees(msgAttitude.pitch)
        yaw_deg = math.degrees(msgAttitude.yaw)
        #print(msgAttitude) #? if u want raw data
        print("Roll: {0} deg, Pitch: {1} deg, Yaw: {2} deg".format(roll_deg, pitch_deg, yaw_deg))
    if msgIMU is not None:
        x_acc, y_acc, z_acc = round(msgIMU.xacc/1000*9.81, 2), round(msgIMU.yacc/1000*9.81, 2), round(msgIMU.zacc/1000*9.81, 2) # converts from mG to G to m/s^2
        x_gyro, y_gyro, z_gyro = round(math.degrees(msgIMU.xgyro), 2), round(math.degrees(msgIMU.ygyro), 2), round(math.degrees(msgIMU.zgyro), 2) # converts from rad/s to deg/s
        #print(msgIMU) #? if u want raw data
        print("Acceleration: X: {0} m/s^2, Y: {1} m/s^2, Z: {2} m/s^2".format(x_acc, y_acc, z_acc))
        print("Gyroscope: X: {0} deg/s, Y: {1} deg/s, Z: {2} deg/s".format(x_gyro, y_gyro, z_gyro)) #! gyro data is unreliable i think the flight controller is smoking
    print("-------------------------------------------------")
    time.sleep(1)

