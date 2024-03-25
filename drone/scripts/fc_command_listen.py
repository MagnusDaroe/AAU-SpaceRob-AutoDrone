#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import time
import threading

class ManualControlListener(Node):
    def __init__(self):
        super().__init__('manual_control_listener')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        print("Connecting to MAVLink...")
        self.the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        self.the_connection.wait_heartbeat()
        print("Connected to MAVLink.")
        time.sleep(2)

        # Set mode stabilize
        self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                     0, 1, 0, 0, 0, 0, 0, 0)
        msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
    
        # Arm the drone
        print("Arming the drone...")
        # Arm the vehicle
        self.the_connection.mav.command_long_send(self.the_connection.target_system,           # Target system ID
            self.the_connection.target_component,       # Target component ID
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command ID
            0,                                     # Confirmation
            1,                                     # Arm
            0,                                     # Empty
            0,                                     # Empty
            0,                                     # Empty
            0,                                     # Empty
            0,                                     # Empty
            0                      # Empty
        )
        msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)

        self.latest_twist_command = Twist()
        self.command_lock = threading.Lock()

    def listener_callback(self, msg):
        with self.command_lock:
            self.latest_twist_command = msg

    def send_twist_command(self):
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            with self.command_lock:
                linear_x = self.latest_twist_command.linear.x
                linear_y = self.latest_twist_command.linear.y
                linear_z = self.latest_twist_command.linear.z
                angular_z = self.latest_twist_command.angular.z

            # Sending MAVLink commands based on manual control inputs
            # Example: Here, assuming linear_z represents throttle control
            print("Sending:" + f"linear_x={linear_x}, linear_y={linear_y}, linear_z={linear_z}, angular_z={angular_z}")
            self.the_connection.mav.manual_control_send(
                self.the_connection.target_system,
                int(linear_x),         # x/pitch
                int(linear_y),         # y/roll
                int(linear_z),  # z/throttle
                int(angular_z),         # yaw
                0          # buttons
            )
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    listener = ManualControlListener()
    threading.Thread(target=listener.send_twist_command, daemon=True).start()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
