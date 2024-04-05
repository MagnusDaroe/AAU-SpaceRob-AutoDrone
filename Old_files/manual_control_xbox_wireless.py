#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import pygame
import math
import numpy as np

class XboxControlNode(Node):
    def __init__(self):
        super().__init__('xbox_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def send_control_command(self, twist):
        self.publisher_.publish(twist)
        self.get_logger().info(f"Throttle: {twist.linear.z}, Linear X: {twist.linear.x}, Linear Y: {twist.linear.y}, Angular Z: {twist.angular.z}")

def control_loop(node):
    while True:
        pygame.event.pump()  # Process event queue
        left_x = node.controller.get_axis(0)
        left_y = node.controller.get_axis(1)  
        right_x = node.controller.get_axis(3)
        right_y = node.controller.get_axis(2)  

        twist = Twist()

        # If any of the values are less than 0.1 of the absolute value, set them to 0
        left_x = left_x if abs(left_x) >= 0.1 else 0
        left_y = left_y if abs(left_y) >= 0.1 else 0
        right_x = right_x if abs(right_x) >= 0.1 else 0
        right_y = right_y if abs(right_y) >= 0.1 else 0

        # Map values.
        twist.linear.x = float(np.interp(right_x, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # linear x from right joystick
        twist.linear.y = float(np.interp(right_y, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # linear y from right joystick
        twist.linear.z = float(np.interp(left_y, (-1, -0.1, 0.1, 1), (1000, 0, 0, -1000)))   # Throttle z from right joystick
        twist.angular.z = float(np.interp(left_x, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # yaw from left joystick's Y-axis
        
        node.send_control_command(twist)

def main(args=None):
    rclpy.init(args=args)
    xbox_control_node = XboxControlNode()
    control_thread = threading.Thread(target=control_loop, args=(xbox_control_node,))
    control_thread.start()
    rclpy.spin(xbox_control_node)
    control_thread.join()
    xbox_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
