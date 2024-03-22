from pymavlink import mavutil
import time
#* https://www.ardusub.com/developers/pymavlink.html#armdisarm-the-vehicle
# Start a connection listening on a UDP port
#* edit the port to the one you are using
the_connection = mavutil.mavlink_connection('/dev/ttyACM0')
#the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
# Wait for the first heartbeat
the_connection.wait_heartbeat()
time.sleep(2)
# Set mode to stabilize
the_connection.mav.command_long_send(
    the_connection.target_system,                   # Target system ID
    the_connection.target_component,                # Target component ID
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,            # Command ID
    0,                                              # Confirmation
    1,                                              # Mode (1 for stabilize)
    0,                                              # Custom Mode
    0,                                              # Custom Submode
    0,                                              # Empty
    0,                                              # Empty
    0,                                              # Empty
    0                                               # Empty
)
msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

# Arm the vehicle
the_connection.mav.command_long_send(
    the_connection.target_system,           # Target system ID
    the_connection.target_component,       # Target component ID
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command ID
    0,                                     # Confirmation
    1,                                     # Arm
    0,                                     # Empty
    0,                                     # Empty
    0,                                     # Empty
    0,                                     # Empty
    0,                                     # Empty
    0                                      # Empty
)
msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

# Set throttle to a non-zero value (assuming you're using a mode that requires manual throttle control)
start_time = time.time()
while time.time() - start_time < 20: #! Maybe not necessary with while loop
    the_connection.mav.manual_control_send(
        the_connection.target_system,
        0, #x/pitch
        0, #y/roll
        500, #z
        0, #yaw
        0) #buttons

# Wait for some time

print("done")
exit()
# Disarm the vehicle
the_connection.mav.command_long_send(
    the_connection.target_system,           # Target system ID
    the_connection.target_component,       # Target component ID
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command ID
    0,                                     # Confirmation
    0,                                     # Disarm
    0,                                     # Empty
    0,                                     # Empty
    0,                                     # Empty
    0,                                     # Empty
    0,                                     # Empty
    0                      # Empty
)
msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

# Close the connection
the_connection.close()
