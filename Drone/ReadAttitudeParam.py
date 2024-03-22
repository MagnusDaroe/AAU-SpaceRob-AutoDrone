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
master = mavutil.mavlink_connection('/dev/ttyACM0')
# Wait a heartbeat before sending commands
master.wait_heartbeat()


# Request attitude
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 0, 1)

# Wait for attitude data
while True:
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    if msg is not None:
        roll_deg = math.degrees(msg.roll)
        pitch_deg = math.degrees(msg.pitch)
        yaw_deg = math.degrees(msg.yaw)
        #print(msg) #? if you want to see the raw data
        print("Roll: {0}, Pitch: {1}, Yaw: {2}".format(roll_deg, pitch_deg, yaw_deg))

