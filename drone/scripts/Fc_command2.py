#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymavlink import mavutil
import time

class ManualControlListener(Node):
    def __init__(self):
        super().__init__('manual_control_listener')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        self.the_connection.wait_heartbeat()
        time.sleep(2)

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        angular_z = msg.angular.z

        # Sending MAVLink commands based on manual control inputs
        # Example: Here, assuming linear_z represents throttle control
        self.the_connection.mav.manual_control_send(
            self.the_connection.target_system,
            0,         # x/pitch
            0,         # y/roll
            int(linear_z * 1000),  # z/throttle
            0,         # yaw
            0          # buttons
        )

def main(args=None):
    rclpy.init(args=args)
    listener = ManualControlListener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
