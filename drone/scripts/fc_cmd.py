#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time
import threading
import math
from drone.msg import DroneCommand, DroneStatus
from drone.srv import Clock
import numpy as np

class RateController:
    def __init__(self, rate):
        self.rate = rate
        self.start_time = time.time()

    def sleep(self):
        elapsed_time = time.time() - self.start_time
        sleep_time = max(0, 1 / self.rate - elapsed_time)
        time.sleep(sleep_time)
        self.start_time = time.time()

class FC_Commander(Node):
    # Class constructor
    def __init__(self):
        super().__init__('fc_command_listener')

        # Initialize the latest command to be sent to the flight controller
        self.fc_command = DroneCommand()
        self.command_lock = threading.Lock()

        # Node parameters
        self.setup_test_parameters()

        # Fc command variables
        self.USB_PORT = '/dev/ttyTHS1'
        self.BAUDRATE = 57600

        # Initialize the connection to the drone
        self.fc_connection = False
        if not self.test_mode:
            self.drone_init()

        # Initialize timout related variables
        self.time_offset = 0
        self.TIME_CALIBRATION_ITERATIONS = 5

        self.TIMEOUT = 0.5
        self.previous_timestamp = 0
        self.last_command_time = self.get_time()
        self.current_time = 0
        
        # Connection variables
        self.battery_check_requested = False
        
        #Reboot
        self.last_reboot = self.get_time()
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

        #Calibrate the clock
        self.calibrate_clock(persistence=False)

        # Set the logging level based on command-line argument or default to INFO
        self.log_level = self.get_logger().get_effective_level()


    # ROS2 specific functions
    def command_callback(self, msg):
        """
        Callback function for the command subscriber\n
        msg: DroneCommand message\n
        This function assigns the received commands to the fc_command variable\n
        """


        # Use self.command_lock to make sure only one thread is accessing the command variables at a time
        with self.command_lock:
            # Assign commands only if they are not NaN

            #self.get_logger().info(f"{msg}")
            if not math.isnan(msg.cmd_mode):
                if msg.identifier == 0:
                    self.fc_command.cmd_estop = msg.cmd_estop
                    self.fc_command.cmd_eland = msg.cmd_eland
                    self.fc_command.cmd_arm = msg.cmd_arm
                    self.fc_command.cmd_mode = msg.cmd_mode
                if self.fc_command.cmd_mode == 0 and msg.identifier == 0 :
                    # Manual mode
                    self.fc_command.cmd_thrust = msg.cmd_thrust
                    self.fc_command.cmd_roll = msg.cmd_roll
                    self.fc_command.cmd_pitch = msg.cmd_pitch
                    self.fc_command.cmd_yaw = msg.cmd_yaw
                elif self.fc_command.cmd_mode == 1 and msg.identifier == 1:
                    # Autonomous mode
                    self.fc_command.cmd_thrust = msg.cmd_auto_thrust
                    self.fc_command.cmd_roll = msg.cmd_auto_roll
                    self.fc_command.cmd_pitch = msg.cmd_auto_pitch
                    self.fc_command.cmd_yaw = msg.cmd_auto_yaw
                elif self.fc_command.cmd_mode == 2 and msg.identifier == 0:
                    # Test mode
                    self.fc_command.cmd_thrust = msg.cmd_thrust
                    self.fc_command.cmd_roll = msg.cmd_roll
                    self.fc_command.cmd_pitch = msg.cmd_pitch
                    self.fc_command.cmd_yaw = msg.cmd_yaw
            
            # Assign the rest of the commands
            self.fc_command.timestamp = msg.timestamp
            
            
    def status_publisher(self):
        """
        Publish the system status\n
        msg: DroneStatus message - {timestamp, battery_ok, battery_percentage, mode, fc_connection}
        """

        # Check the battery voltage - request a battery check
        self.battery_check_requested = True

        # Publish the status. create a DroneStatus msg object
        msg = DroneStatus()
        # Set the message values
        msg.timestamp = self.get_time()
        msg.battery_ok = self.battery_ok
        
        # Calculate the battery percentage
        if not self.test_mode:
            try:
                msg.battery_percentage = (self.battery_voltage - self.BATTERY_MIN_VOLTAGE) / (self.BATTERY_MAX_VOLTAGE - self.BATTERY_MIN_VOLTAGE) * 100
                msg.battery_check_timestamp = self.battery_check_timestamp
            except ZeroDivisionError:
                msg.battery_percentage = 0

        # Set the mode and connection status
        msg.mode = self.fc_command.cmd_mode if not self.fc_command.cmd_estop else 2
        msg.fc_connection = True if self.fc_connection else False
        
        # Publish the message
        self.publisher_status.publish(msg)

    def setup_test_parameters(self):
        # Define the test mode
        self.test_mode = False

        # Check battery voltage
        self.do_battery_check = True
        
        "Test parameters - Currently the rest of the code is not used"
        
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
     
    def calibrate_clock(self, persistence=False):
        """
        Calibrate the clock to synchronize the time between the GCS and the computer
        """
        # save current time
        self.get_logger().info("Calibrating the clock...")
        self.client = self.create_client(Clock, '/sync_clock')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object
        request = Clock.Request()

        # Server time list and client time list
        server_time = np.array([])
        client_time_send = np.array([])
        client_time_received = np.array([])

        # Send a service to the GCS to get the current time
        i = 0
        self.time_offset = 0  # Initialize time offset to 0
        while i < self.TIME_CALIBRATION_ITERATIONS:
            try:
                # Save the current time
                client_time_send = np.append(client_time_send, time.time())

                # Send the request
                future = self.client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    # Save the response time
                    response = future.result()
                    server_time = np.append(server_time, response.server_time)
                    # Save the time received
                    client_time_received = np.append(client_time_received, time.time())

                else:
                    self.get_logger().error(f"Failed to get response for request {i + 1}")
            except Exception as e:
                self.get_logger().error(f"Failed to send request {i + 1}: {e}")
                if not persistence:
                    break
            i += 1
        # Calculate the time offset
        self.time_offset = np.mean(server_time - (client_time_send + client_time_received) / 2)

        if np.isnan(self.time_offset):
            self.get_logger().error("Failed to calibrate the clock")
            self.time_offset = 0
        self.get_logger().info(f"Time offset: {self.time_offset}")

        self.get_logger().info(f"Clock calibration completed: Time offset is {self.time_offset} seconds.")

    def get_time(self):
        """
        Get the current time\n
        return: current time in seconds - Calibrated from the GCS\n
        """
        return time.time() + self.time_offset if self.time_offset else time.time()


    # Drone functions
    def drone_init(self):
        """
        Start the connection to the flight controller
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
        
        # Check if the mode is set
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
        """
        Reboot the drone.\n
        Keep sending the reboot command until the drone reboots
        """

        # Reboot the drone
        self.get_logger().info("Rebooting the drone...")
        while not self.test_mode and self.get_time() - self.last_reboot < self.MIN_REBOOT_INTERVAL:
            time.sleep(1)

        # Reboot the drone
        if not self.test_mode and self.get_time() - self.last_reboot > self.MIN_REBOOT_INTERVAL:
            Ack = False

            # Send the reboot command - Keep sending the command until the drone reboots
            while not Ack:    
                self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                            0, 1, 0, 0, 0, 0, 0, 0)
                Ack = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
                if Ack.result == 0:
                    self.get_logger().warn("Failed to reboot. Trying again...")

            # Wait for the drone to reboot        
            time.sleep(8)
            self.get_logger().info("Rebooted the drone")

            # Start the drone again
            self.drone_init()
        else:
            # If the drone is rebooted too soon, wait for the reboot interval to expire
            if not self.test_mode: 
                self.get_logger().warn("Rebooting too soon. Waiting for the reboot interval to expire")

    def fc_commander(self, updaterate=100):
        """
        Main function for the flight controller commander \n\n
        Param:\n
        * updaterate: Update rate in Hz\n

        This function sends the latest command to the flight controller\n
        Controls the mode of the drone
        """
        # Initialize your rate controller
        rate_controller = RateController(updaterate)

        # Main loop. Rclpy.ok() returns False when the node is shutdown
        while rclpy.ok():
            # Check for estop condition
            if self.fc_command.cmd_estop == True:
                # Emergency stop
                self.emergency_stop()
                # Sleep to keep the update rate
                continue
            elif self.fc_command.cmd_eland == True:
                # Emergency land
                self.safe_mode()
                # Sleep to keep the update rate
                continue
            
            # Check the battery voltage if requested
            if self.battery_check_requested:
                self.get_logger().info("Battery check requested")
                self.check_battery()
                self.battery_check_requested = False
            
            # Verify if the drone can keep operating
            if not self.battery_ok:
                # Battery voltage too low. Emergency stop activated
                self.get_logger().warn("Battery voltage too low. Emergency stop activated. Recharge the battery and restart the drone. ")
                self.emergency_stop() # Implement an emergency land.
            # Check if the drone is armed
            elif self.fc_command.cmd_arm == 1:
                # Send command
                self.flight_mode()
                
                # Sleep to keep the update rate
                rate_controller.sleep()
            else:
                self.get_logger().info("Waiting for arm command")
                while not self.fc_command.cmd_arm and not self.fc_command.cmd_estop:
                    time.sleep(0.5)
                    # Check if estop is activated
                    if self.fc_command.cmd_estop:
                        self.emergency_stop()
                        break  
                self.reset_cmd()
                
                # Arm the drone again
                if not self.test_mode:
                    self.drone_arm()
    
    def flight_mode(self):
        """
        Drone flight mode. Sends the latest command to the flight controller
        """
        # Use self.command_lock to make sure only one thread is accessing the command variables at a time
        with self.command_lock:
            if self.fc_command.cmd_mode not in [0,1,2]:
                self.get_logger().fatal("Drone mode not recognized")
                # Shut down
                rclpy.shutdown()
                
            if self.fc_command.cmd_mode == 2:
                #Testmode - If yaw is positive, set it to x, else set it to 0. If yaw is negative, set it to -x
                x = 400
                if self.fc_command.cmd_roll:
                    if self.fc_command.cmd_roll > 0:
                        self.fc_command.cmd_roll = float(x)
                    elif self.fc_command.cmd_roll < 0:
                        self.fc_command.cmd_roll = float(-x)
                    else:
                        self.fc_command.cmd_roll = float(0)
             
            # Update command variables - if no new command is received, the previous command is sent
            timestamp = self.fc_command.timestamp
            
            # Check if the command is new or if the timeout has expired
            self.current_time = self.get_time()
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
        # Makes sure the correct command type is sent
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
        
        # Disarm the drone
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
        self.get_logger().info("Checking battery voltage")

        # Request the battery voltage
        self.the_connection.mav.request_data_stream_send(
            self.the_connection.target_system,
            self.the_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            1,
            1
        )
        # Wait for the battery voltage
        msg = self.the_connection.recv_match(type='SYS_STATUS', blocking=True)
        
        # Get the battery voltage in volts
        self.battery_voltage = msg.voltage_battery / 1000.0
        
        # Check if the battery voltage is within the operating range
        self.battery_check_timestamp = self.get_time()
        self.battery_ok = True if self.battery_voltage > self.BATTERY_MIN_OP_VOLTAGE else False 
        self.get_logger().info(f"Battery voltage: {self.battery_voltage}, Battery ok: {self.battery_ok}")

    def safe_mode(self):
        """
        Safe mode. The drone will land and disarm if the mode is set to safe mode
        """
        decrement_thrust = 500
        land_thrust = 100
        decrement = 1
        self.fc_command.cmd_roll = float(0)
        self.fc_command.cmd_pitch = float(0)
        self.fc_command.cmd_yaw = float(0)
        
        # Decrement start_thrust a set amount, until a lower bound is reached
        while decrement_thrust > land_thrust:
            if self.fc_command.cmd_estop == True:
                # Emergency stop
                self.emergency_stop()
                break

            decrement_thrust = decrement_thrust - decrement

            if decrement_thrust < land_thrust:
                
                self.fc_command.cmd_thrust  = float(0)
            else:
                self.fc_command.cmd_thrust = float(decrement_thrust)
                
            # Send command.    
            self.flight_cmd()

        # Go into emergency stop mode, to complete the emergency landing
        self.emergency_stop()

    def reset_cmd(self):
        """
        Reset the command variables
        """

        # Reset the command variables to 0
        self.fc_command.cmd_thrust = float(0)
        self.fc_command.cmd_roll = float(0)
        self.fc_command.cmd_pitch = float(0)
        self.fc_command.cmd_yaw = float(0)
            
def main(args=None):
    # Initialize the node
    rclpy.init(args=args)
    
    # Create the node - daemon=True to stop the thread when the main thread stops
    FC_node = FC_Commander()
    threading.Thread(target=FC_node.fc_commander, daemon=True).start()

    # Spin the node
    rclpy.spin(FC_node)

    # Destroy the node when code is stopped
    FC_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
