#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time
import threading
import math
from drone.msg import DroneCommand, DroneStatus

# Maybe at ntp to the drone to sync the time

# Set test_mode to True to run the script without a drone
test_mode = False

# Check battery voltage
do_battery_check = True


class FC_Commander(Node):
    def __init__(self):
        super().__init__('fc_command_listener')

        # Define subscriber
        self.subscription_commands = self.create_subscription(
            DroneCommand,
            '/cmd_fc',
            self.command_callback,
            10
        )

        # Define publisher
        self.publisher_autonomous = self.create_publisher(
            DroneStatus,
            '/status_fc',
            10
        )
        #self.publish_timer = self.create_timer(5, self.status_publisher)

        while not DroneCommand.cmd_arm:
            print("Waiting for arm command", end='\r')
            time.sleep(0.5)

        # Fc command variables
        self.usb_port = '/dev/ttyTHS1'
        self.baudrate = 57600

        # Initialize the connection to the drone
        if not test_mode:
            self.drone_init()

        # Initialize timout related variables
        self.previous_timestamp = 0
        self.last_command_time = time.time()
        self.current_time = 0
        self.timeout = 0.5

        # Initialize the battery check
        self.battery_ok = True
        if not test_mode:
            if do_battery_check:
                print("Initial battery check")
                self.battery_ok = False
                self.battery_check_interval = 10
                self.battery_min_op_voltage = 13.00
                self.battery_min_voltage = 12.00
                self.battery_max_voltage = 16.80
                self.check_battery()
        
        # Initialize the latest command to be sent to the flight controller
        self.fc_command = DroneCommand()
        self.command_lock = threading.Lock()

    def command_callback(self, msg):
        with self.command_lock:
            # Assign commands only if they are not NaN
            if not math.isnan(msg.cmd_mode):
                self.fc_command.cmd_mode = msg.cmd_mode
                if msg.cmd_mode == 0:
                    self.fc_command.cmd_thrust = msg.cmd_thrust
                    self.fc_command.cmd_roll = msg.cmd_roll
                    self.fc_command.cmd_pitch = msg.cmd_pitch
                    self.fc_command.cmd_yaw = msg.cmd_yaw
                elif msg.cmd_mode == 1:
                    self.fc_command.cmd_thrust = msg.cmd_auto_thrust
                    self.fc_command.cmd_roll = msg.cmd_auto_roll
                    self.fc_command.cmd_pitch = msg.cmd_auto_pitch
                    self.fc_command.cmd_yaw = msg.cmd_auto_yaw
            
             # Decide which command to send to the flight controller
            self.fc_command.timestamp = msg.timestamp
            self.fc_command.cmd_estop = msg.cmd_estop
            self.fc_command.cmd_arm = msg.cmd_arm

            if True:
                try:

                    with open('thrust_log_received.csv', 'a') as f:
                        f.write(f"{time.time()},{self.fc_command.cmd_thrust}\n")
                except Exception as e:
                    print(f"Error occurred while writing to thrust_log.csv: {e}")

    def status_publisher(self):
        """
        Publish the system status
        """
        # Check the battery level
        if not test_mode and do_battery_check and time.time() - self.last_command_time > self.battery_check_interval:
            self.check_battery()
        

        msg = DroneStatus()
        msg.timestamp = time.time()
        msg.battery_ok = self.battery_ok
        msg.battery_percentage = (self.battery_max_voltage - self.battery_min_voltage) / (self.battery_max_voltage - self.battery_min_voltage) * 100
        msg.drone_mode = self.fc_command.cmd_mode if not self.fc_command.cmd_estop else 2

        self.publish_status.publish(msg)

    def drone_init(self):
        """
        Start the connection to the flight controller and arm the drone
        """

        print("Connecting to MAVLink...")
        self.the_connection = mavutil.mavlink_connection(self.usb_port,baud=self.baudrate)
        self.the_connection.wait_heartbeat()
        print("Connected to MAVLink.")
        time.sleep(2)

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
            
            if self.fc_command.cmd_arm == 1 and not self.fc_command.cmd_estop == 1 and self.battery_ok:
                # Check if the drone is in manual or autonomous mode
                if self.fc_command.cmd_mode == 0 or self.fc_command.cmd_mode == 1:
                    self.flight_mode()
                else:
                    print("Undefined mode. landing the drone. ", end='\r')
                    self.safe_mode()

                # Sleep to keep the update rate
                rate.sleep()
            elif not self.battery_ok:
                print("Battery voltage too low. Emergency stop activated. Recharge the battery and restart the drone. ", end='\r')
                self.emergency_stop() # Implement an emergency land.
            elif self.fc_command.cmd_estop == 1:
                # Emergency stop
                self.emergency_stop()
            else:
                print("Waiting for arm command                                               ", end='\r')
                while not self.fc_command.cmd_arm and self.fc_command.cmd_estop or not self.fc_command.cmd_arm:
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
            timestamp = self.fc_command.timestamp
            
            # Check if the command is new or if the timeout has expired
            self.current_time = time.time()
            if self.previous_timestamp != timestamp or self.current_time - self.last_command_time <= self.timeout:
                # Send the command to the flight controller
                self.flight_cmd()

                # Update last_command_time only when a new command is sent
                if self.previous_timestamp != timestamp:
                    self.last_command_time = self.current_time  
            else:
                # If the timeout has expired, send a stop command. Maybe implement a safemode in the future
                print("No new command received. Going into safe mode.", end = '\r')
                self.safe_mode()

        self.previous_timestamp = timestamp

    def flight_cmd(self):
        """
        Send flight command to the drone
        """
        log = True
        roll = self.fc_command.cmd_roll
        pitch = self.fc_command.cmd_pitch
        yaw = self.fc_command.cmd_yaw
        thrust = self.fc_command.cmd_thrust

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

        if log:
            try:

                with open('thrust_log.csv', 'a') as f:
                    f.write(f"{time.time()},{thrust}\n")
            except Exception as e:
                print(f"Error occurred while writing to thrust_log.csv: {e}")

            
    def emergency_stop(self):
        """
        Emergency stop mode. Disarms the drone and waits for the arm and estop commands to be released
        """
        while self.fc_command.cmd_arm == 1 or self.fc_command.cmd_estop == 1 or self.fc_command.cmd_thrust != 0:
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
            print("Emergency stop mode activted. Release Arm, Estop and set throttle to 0, to regain control                                                       ", end='\r')
            time.sleep(0.5)
        print("Emergency stop command released                                     ", end='\r')
    
    def check_battery(self):
        """
        Check the battery level of the drone
        """
        voltage_sum = 0
        for _ in range(3):
            self.the_connection.mav.request_data_stream_send(
                self.the_connection.target_system,           # Target system ID
                self.the_connection.target_component,       # Target component ID
                mavutil.mavlink.MAV_DATA_STREAM_ALL,   # All streams
                1,                                     # Enable
                1                                      # Rate (Hz)
            )
            msg = self.the_connection.recv_match(type='SYS_STATUS', blocking=True)
            voltage_sum += msg.voltage_battery / 1000.0
            
        voltage_avg = voltage_sum / 3
        
        self.battery_ok = True if voltage_avg > self.battery_min_op_voltage else False 
        print(f"Battery voltage: {voltage_avg}")
        if self.battery_ok:
            print(f"Battery ok")
        else:
            
            print("Battery not good")

        time.sleep(2)

    def safe_mode(self):
        """
        Safe mode. The drone will land and disarm if the mode is set to safe mode
        """
        decrement_thrust = self.fc_command.cmd_thrust
        land_thrust = 400
        decrement = 2
        self.fc_command.cmd_roll = float(0)
        self.fc_command.cmd_pitch = float(0)
        self.fc_command.cmd_yaw = float(0)
        # decrement start_thrust a set amount, until a lower bound is reached
        while decrement_thrust > land_thrust:
            decrement_thrust = decrement_thrust - decrement

            if decrement_thrust < land_thrust:
                
                self.fc_command.cmd_thrust  = float(0)
            else:
                self.fc_command.cmd_thrust = float(decrement_thrust)
                
            # Send command.    
            self.flight_cmd()
            

def main(args=None):
    rclpy.init(args=args)
    listener = FC_Commander()
    threading.Thread(target=listener.fc_commander, daemon=True).start()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
False