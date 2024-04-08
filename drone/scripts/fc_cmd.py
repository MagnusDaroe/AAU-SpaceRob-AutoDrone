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
            self.drone_init()

        # Initialize timout related variables
        self.previous_timestamp = 0
        self.last_command_time = time.time()
        self.current_time = 0
        self.timeout = 0.5

        # Fc command variables
        self.usb_port = '/dev/ttyTHS1'
        self.baudrate = 57600

        # Initialize the latest command to be sent to the flight controller
        self.latest_fc_command = DroneCommand()
        self.command_lock = threading.Lock()

    def listener_callback(self, msg):
        with self.command_lock:
            self.latest_fc_command = msg

    def drone_init(self):
        """
        Start the connection to the flight controller and arm the drone
        """

        print("Connecting to MAVLink...")
        self.the_connection = mavutil.mavlink_connection(self.usb_port, baud=self.baudrate)
        self.the_connection.wait_heartbeat()
        print("Connected to MAVLink.")
        time.sleep(2)

        # Arm the drone
        self.drone_arm()

    def drone_arm(self):
        """
        Arm the drone. Set mode to stabilize and arm the drone
        """
        # Set mode stabilize
        self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                    0, 1, 0, 0, 0, 0, 0, 0)
        Ack = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
        if Ack.result == 0:
            print("Mode set to stabilize")
        else:
            print("Failed to set mode")
            exit()

        # Arm the drone. It must be done two times for some reason.
        print("Arming the drone...")
        # Arm the vehicle - run it two times
        for _ in range(2):
            self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                        0, 1, 0, 0, 0, 0, 0, 0)
            time.sleep(1)

        Ack = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
        if Ack.result == 0:
            print("Armed the drone")
        else:
            print("Failed arm")
            exit()

    def fc_commander(self, updaterate=50):
        """
        Main function for the flight controller commander. \n
        """
        # Set the rate of the commander
        rate = self.create_rate(updaterate)
        
        # Main loop
        while rclpy.ok():
            if self.latest_fc_command.cmd_arm == 1 and not self.latest_fc_command.cmd_estop == 1:
                # Call mode which the drone flies in
                self.flight_mode()
                # Sleep to keep the update rate
                rate.sleep()
            elif self.latest_fc_command.cmd_estop == 1:
                # Emergency stop
                self.emergency_stop()
            else:
                print("Waiting for arm command                                               ", end='\r')
                while not self.latest_fc_command.cmd_arm and self.latest_fc_command.cmd_estop or not self.latest_fc_command.cmd_arm:
                    time.sleep(0.5)
                
                # Arm the drone again
                if not test_mode:
                    self.drone_arm()
    
    def flight_mode(self):
        """
        Drone flight mode. Sends the latest command to the flight controller
        """

        with self.command_lock:
                # Update command variables - if no new command is received, the previous command is sent
                timestamp = self.latest_fc_command.timestamp
                roll = self.latest_fc_command.cmd_roll
                pitch = self.latest_fc_command.cmd_pitch
                yaw = self.latest_fc_command.cmd_yaw
                thrust = self.latest_fc_command.cmd_thrust

                # Check if the command is new or if the timeout has expired
                self.current_time = time.time()
                if self.previous_timestamp != timestamp or self.current_time - self.last_command_time <= self.timeout:
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
                    if self.previous_timestamp != timestamp:
                        self.last_command_time = self.current_time  
                else:
                    # If the timeout has expired, send a stop command. Maybe implement a safemode in the future
                    print("No new command received", end = '\r')
                    roll = 0
                    pitch = 0
                    yaw = 0
                    thrust = 0

        self.previous_timestamp = timestamp

    def emergency_stop(self):
        """
        Emergency stop mode. Disarms the drone and waits for the arm and estop commands to be released
        """
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
    
    

def main(args=None):
    rclpy.init(args=args)
    listener = FC_Commander()
    threading.Thread(target=listener.fc_commander, daemon=True).start()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
