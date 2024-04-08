#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time
import threading
from drone.msg import DroneCommand

# Set test_mode to True to run the script without a drone
test_mode = False

class FC_Commander(Node):
    def __init__(self):
        super().__init__('fc_command_listener')
        self.subscription = self.create_subscription(
            DroneCommand,
            '/cmd_fc',
            self.listener_callback,
            10
        )
        while not DroneCommand.cmd_arm:
            print("Waiting for arm command", end='\r')
            time.sleep(0.5)

        # Initialize the connection to the drone
        if not test_mode:
            self.the_connection = Drone_init(self)

        # Initialize the latest command to be sent to the flight controller
        self.latest_fc_command = DroneCommand()
        self.command_lock = threading.Lock()

    def listener_callback(self, msg):
        with self.command_lock:
            self.latest_fc_command = msg

    def fc_commander(self, updaterate=50, timeout=0.5):
        """
        Allowed msg definitions:
        uint64 timestamp
        uint8 cmd_arm
        uint8 cmd_mode
        float32 cmd_pitch
        float32 cmd_roll
        float32 cmd_yaw
        float32 cmd_thrust
        """
        
        # Set the rate of the commander
        rate = self.create_rate(updaterate)

        # Initialize timout related variables
        previous_timestamp = 0
        last_command_time = time.time()
        
        # Main loop
        while rclpy.ok():
            if self.latest_fc_command.cmd_arm == 1 and not self.latest_fc_command.cmd_estop == 1:
                with self.command_lock:
                    # Update command variables - if no new command is received, the previous command is sent
                    timestamp = self.latest_fc_command.timestamp
                    roll = self.latest_fc_command.cmd_roll
                    pitch = self.latest_fc_command.cmd_pitch
                    yaw = self.latest_fc_command.cmd_yaw
                    thrust = self.latest_fc_command.cmd_thrust

                    # Check if the command is new or if the timeout has expired
                    current_time = time.time()
                    if previous_timestamp != timestamp or current_time - last_command_time <= timeout:
                        # Send the command to the flight controller
                        
                        if not test_mode:
                            self.the_connection.mav.manual_control_send(
                                self.the_connection.target_system,
                                int(roll),
                                int(pitch),
                                int(thrust),
                                int(yaw),
                                0
                            )
                        print("Sending:" + f"Roll={roll}, Pitch={pitch}, Thrust={thrust}, Yaw={yaw}", end='\r')
                        
                        # Update last_command_time only when a new command is sent
                        if previous_timestamp != timestamp:
                            last_command_time = current_time  
                    else:
                        # If the timeout has expired, send a stop command. Maybe implement a safemode in the future
                        print("No new command received", end = '\r')
                        roll = 0
                        pitch = 0
                        yaw = 0
                        thrust = 0

                previous_timestamp = timestamp
                rate.sleep()
            elif self.latest_fc_command.cmd_estop == 1:
                while self.latest_fc_command.cmd_arm == 1 or self.latest_fc_command.cmd_estop == 1:
                    if not test_mode:
                         self.the_connection.mav.command_long_send(self.the_connection.target_system,           # Target system ID
                            self.the_connection.target_component,       # Target component ID
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command ID
                            0,                                     # Confirmation
                            0,                                     # Disarm
                            21196,                                 # Force disarm
                            0,                                     # Empty
                            0,                                     # Empty
                            0,                                     # Empty
                            0,                                     # Empty
                            0                      # Empty
                            )
                    print("Emergency stop mode activted. Release Arm and Estop to regain control                                                       ", end='\r')
                    time.sleep(0.5)
                print("Emergency stop command released                                     ", end='\r')
            else:
                print("Waiting for arm command                                               ", end='\r')
                while not self.latest_fc_command.cmd_arm and self.latest_fc_command.cmd_estop or not self.latest_fc_command.cmd_arm:
                    time.sleep(0.5)
                
                # Arm the drone again
                if not test_mode:
                    drone_arm(self)

def Drone_init(self):
    print("Connecting to MAVLink...")
    self.the_connection = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)
    self.the_connection.wait_heartbeat()
    print("Connected to MAVLink.")
    time.sleep(2)

    # Arm the drone
    drone_arm(self)

    return self.the_connection

def drone_arm(self):
    # Set mode stabilize
    self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                 0, 1, 0, 0, 0, 0, 0, 0)
    Ack = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    if Ack.result == 0:
        print("Mode set to stabilize")
    else:
        print("Failed to set mode")
        exit()

    # Arm the drone
    print("Arming the drone...")
    # Arm the vehicle - run it two times
    for _ in range(2):
        self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                     0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1)

    msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    if Ack.result == 0:
        print("Armed the drone")
    else:
        print("Failed arm")
        exit()

def main(args=None):
    rclpy.init(args=args)
    listener = FC_Commander()
    threading.Thread(target=listener.fc_commander, daemon=True).start()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
