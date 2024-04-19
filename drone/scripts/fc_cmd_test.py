#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time
import threading
import math
import logging
from drone.msg import DroneCommand, DroneStatus


class FC_Commander(Node):
    def __init__(self):
        super().__init__('fc_command_listener')

        # Node parameters
        self.setup_test_parameters()



        # Define subscriber
        self.subscription_commands = self.create_subscription(
            DroneCommand,
            '/cmd_fc',
            self.command_callback,
            10
        )

        # Define publisher
        self.publisher_status= self.create_publisher(
            DroneStatus,
            '/status_fc',
            10
        )
        self.publish_timer = self.create_timer(5, self.status_publisher)


        # Fc command variables
        self.USB_PORT = '/dev/ttyTHS1'
        self.BAUDRATE = 57600

        # Initialize the connection to the drone
        self.fc_connection = False
        if not self.test_mode:
            self.drone_init()

        # Initialize timout related variables
        self.TIMEOUT = 0.5
        self.previous_timestamp = 0
        self.last_command_time = time.time()
        self.current_time = 0
        
        # Connection variables
        self.battery_check_requested = False
        
        #Reboot
        self.last_reboot = time.time()
        self.MIN_REBOOT_INTERVAL = 10

        # Initialize the battery check
        self.battery_ok = True
        if not self.test_mode:
            if self.do_battery_check:
                print("Initial battery check")
                self.battery_voltage = 15.0
                self.battery_ok = False
                self.battery_check_interval = 10
                self.BATTERY_MIN_OP_VOLTAGE = 13.00
                self.BATTERY_MIN_VOLTAGE = 12.00
                self.BATTERY_MAX_VOLTAGE = 16.8
                self.check_battery()
        
        # Initialize the latest command to be sent to the flight controller
        self.fc_command = DroneCommand()
        self.command_lock = threading.Lock()

        # Set the logging level based on command-line argument or default to INFO
        self.log_level = self.get_logger().get_effective_level()

    def setup_test_parameters(self):
        # Define a dictionary mapping test types to attributes
        test_attributes = {
            'pitch': ('test_pitch', 'test_pitch_value'),
            'roll': ('test_roll', 'test_roll_value'),
            'yaw': ('test_yaw', 'test_yaw_value'),
            'thrust': ('test_thrust', 'test_thrust_value')
        }

        # Node parameters
        self.test_type = self.get_parameter('test_type').value if self.has_parameter('test_type') else None

        # Initialize all test flags to False
        self.test_pitch = False
        self.test_roll = False
        self.test_yaw = False
        self.test_thrust = False

        # Set the test values based on test type
        if self.test_type in test_attributes:
            test_flag, value_param = test_attributes[self.test_type]
            setattr(self, test_flag, True)
            setattr(self, value_param, self.get_parameter(value_param).value if self.has_parameter(value_param) else 'nan')

        # Make the test attributes available to the class
        self.test_attributes = test_attributes

               
        # Set self.test_mode to True to run the script without a drone
        self.test_mode = False

        # Check battery voltage
        self.do_battery_check = True



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
                elif msg.cmd_mode == 2:
                    self.fc_command.cmd_thrust = msg.cmd_thrust
                    self.fc_command.cmd_roll = msg.cmd_roll
                    self.fc_command.cmd_pitch = msg.cmd_pitch
                    self.fc_command.cmd_yaw = msg.cmd_yaw
            
             # Decide which command to send to the flight controller
            self.fc_command.timestamp = msg.timestamp
            self.fc_command.cmd_estop = msg.cmd_estop
            self.fc_command.cmd_arm = msg.cmd_arm

    def status_publisher(self):
        """
        Publish the system status
        """

        self.battery_check_requested = True
        msg = DroneStatus()
        msg.timestamp = time.time()
        msg.battery_ok = self.battery_ok
        
        if not self.test_mode:
            msg.battery_percentage = (self.battery_voltage - self.BATTERY_MIN_VOLTAGE) / (self.BATTERY_MAX_VOLTAGE - self.BATTERY_MIN_VOLTAGE) * 100
            msg.battery_check_timestamp = self.battery_check_timestamp

        msg.mode = self.fc_command.cmd_mode if not self.fc_command.cmd_estop else 2
        msg.fc_connection = True if self.fc_connection else False
        

        self.publisher_status.publish(msg)

    def drone_init(self):
        """
        Start the connection to the flight controller and arm the drone
        """

        self.get_logger().info("Connecting to MAVLink...")
        self.the_connection = mavutil.mavlink_connection(self.USB_PORT,baud=self.BAUDRATE)
        self.the_connection.wait_heartbeat()
        self.get_logger().info("Connected to MAVLink.")
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
            self.get_logger().info("Mode set to stabilize")
        else:
            self.get_logger().fatal("Failed to set mode. Trying to reboot the drone...")
            self.drone_reboot()


        # Arm the drone. It must be done two times for some reason.
        self.get_logger().info("Arming the drone...")
        # Arm the vehicle - run it two times
        for _ in range(2):
            self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                        0, 1, 0, 0, 0, 0, 0, 0)
            time.sleep(1)

        Ack = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
        if Ack.result == 0:
            self.get_logger().info("Armed the drone")
        else:
            self.get_logger().fatal("Failed arm. Trying to reboot the drone...")
            self.drone_reboot()

    def drone_reboot(self):
        # Reboot the drone
        self.get_logger().info("Rebooting the drone...")
        while not self.test_mode and time.time() - self.last_reboot < self.MIN_REBOOT_INTERVAL:
            time.sleep(1)

        if not self.test_mode and time.time() - self.last_reboot > self.MIN_REBOOT_INTERVAL:
            Ack = False

            while not Ack:    
                self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                            0, 1, 0, 0, 0, 0, 0, 0)
                Ack = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
                if Ack.result == 0:
                    self.get_logger().warn("Failed to reboot. Trying again...")
                    
            time.sleep(8)
            self.get_logger().info("Rebooted the drone")

            # Start the drone again
            self.drone_init()
        else:
            if not self.test_mode: 
                self.get_logger().warn("Rebooting too soon. Waiting for the reboot interval to expire")

    def fc_commander(self, updaterate=50):
        """
        Main function for the flight controller commander. \n
        """
        # Set the rate of the commander
        rate = self.create_rate(updaterate)
        
        # Main loop
        while rclpy.ok():
            if self.battery_check_requested:
                self.get_logger().info("Battery check requested")
                self.check_battery()
                self.battery_check_requested = False

            
            if self.fc_command.cmd_arm == 1 and not self.fc_command.cmd_estop == 1 and self.battery_ok:
                # Check if the drone is in manual, autonomous mode and test mode
                if self.fc_command.cmd_mode == 0:
                    self.flight_mode()
                elif self.fc_command.cmd_mode == 1:
                    self.flight_mode()
                elif self.fc_command.cmd_mode == 2:
                    # If yaw is positive, set it to x, else set it to 0. If yaw is negative, set it to -x
                    x = 400
                    if self.fc_command.cmd_pitch:
                        if self.fc_command.cmd_pitch > 0:
                            self.fc_command.cmd_pitch = float(x)
                        elif self.fc_command.cmd_pitch < 0:
                            self.fc_command.cmd_pitch = float(-x)
                        else:
                            self.fc_command.cmd_pitch = float(0)
                    self.flight_mode()
                else:
                    self.get_logger().fatal("Drone mode not recognized")
                    # Shut down
                    rclpy.shutdown()

                # Sleep to keep the update rate
                rate.sleep()
            elif not self.battery_ok:
                self.get_logger().warn("Battery voltage too low. Emergency stop activated. Recharge the battery and restart the drone. ")
                self.emergency_stop() # Implement an emergency land.
            elif self.fc_command.cmd_estop == 1:
                # Emergency stop
                self.emergency_stop()
            else:
                self.get_logger().info("Waiting for arm command")
                while not self.fc_command.cmd_arm and self.fc_command.cmd_estop or not self.fc_command.cmd_arm:
                    time.sleep(0.5)
                self.reset_cmd()
                
                # Arm the drone again
                if not self.test_mode:
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
            if self.previous_timestamp != timestamp or self.current_time - self.last_command_time <= self.TIMEOUT:
                # Send the command to the flight controller
                self.flight_cmd()

                # Update last_command_time only when a new command is sent
                if self.previous_timestamp != timestamp:
                    self.last_command_time = self.current_time  
            else:
                # If the timeout has expired, send a stop command. Maybe implement a safemode in the future
                self.get_logger().warn("No new command received. Going into safe mode.")
                self.safe_mode()
                self.fc_command.cmd_estop = 1


        self.previous_timestamp = timestamp

    def flight_cmd(self):
        """
        Send flight command to the drone
        """
   
        if not self.test_mode:
                self.the_connection.mav.manual_control_send(
                    self.the_connection.target_system,
                    int(self.fc_command.cmd_pitch),
                    int(self.fc_command.cmd_roll),
                    int(self.fc_command.cmd_thrust),
                    int(self.fc_command.cmd_yaw),
                    0
                )
    
        # Log the command values
        self.get_logger().info(f"Sending: Roll={int(self.fc_command.cmd_roll)}, Pitch={int(self.fc_command.cmd_pitch)}, Thrust={int(self.fc_command.cmd_thrust)}, Yaw={int(self.fc_command.cmd_yaw)}")
       
    def emergency_stop(self):
        """
        Emergency stop mode. Disarms the drone and waits for the arm and estop commands to be released
        """
        self.get_logger().warn("Emergency stop mode activted. Release Arm, Estop and set throttle to 0, to regain control")
        while self.fc_command.cmd_arm == 1 or self.fc_command.cmd_estop == 1 or self.fc_command.cmd_thrust != 0:
            if not self.test_mode:
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
            time.sleep(0.5)
        self.get_logger().info("Emergency stop command released")
    
    def check_battery(self):
        """
        Check the battery level of the drone
        """
        # Request the battery voltage
        self.the_connection.mav.request_data_stream_send(
            self.the_connection.target_system,
            self.the_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            1,
            1
        )
        msg = self.the_connection.recv_match(type='SYS_STATUS', blocking=True)
        self.battery_voltage = msg.voltage_battery / 1000.0
        
        self.battery_check_timestamp = time.time()
        self.battery_ok = True if self.battery_voltage > self.BATTERY_MIN_OP_VOLTAGE else False 
        self.get_logger().info(f"Battery voltage: {self.battery_voltage}, Battery ok: {self.battery_ok}")

    def safe_mode(self):
        """
        Safe mode. The drone will land and disarm if the mode is set to safe mode
        """
        decrement_thrust = self.fc_command.cmd_thrust
        land_thrust = 400
        decrement = 1
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

    def reset_cmd(self):
        """
        Reset the command variables
        """
        self.fc_command.cmd_thrust = float(0)
        self.fc_command.cmd_roll = float(0)
        self.fc_command.cmd_pitch = float(0)
        self.fc_command.cmd_yaw = float(0)
            
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