from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
master = mavutil.mavlink_connection('/dev/ttyACM0')
#master = mavutil.mavlink_connection('udpin:localhost:14551')
# Wait for the first heartbeat
master.wait_heartbeat()
time.sleep(2)
# Set mode to stabilize
master.mav.command_long_send(
    master.target_system,                   # Target system ID
    master.target_component,                # Target component ID
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
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

# Arm the vehicle
master.mav.command_long_send(
    master.target_system,           # Target system ID
    master.target_component,       # Target component ID
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
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Disarm
# master.arducopter_disarm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

# wait until disarming confirmed
print("Waiting for the vehicle to unarm")
master.motors_disarmed_wait()
print('Unarmed!')