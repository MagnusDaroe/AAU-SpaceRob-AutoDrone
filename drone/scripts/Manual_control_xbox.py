#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
from inputs import get_gamepad
import math
import numpy as np

class XboxController:
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        # Return joystick values
        left_x = self.LeftJoystickX
        left_y = self.LeftJoystickY
        right_x = self.RightJoystickX
        right_y = self.RightJoystickY
        return left_x, left_y, right_x, right_y

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL  # Normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL  # Normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL  # Normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL  # Normalize between -1 and 1

class XboxControlNode(Node):
    def __init__(self):
        super().__init__('xbox_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.controller = XboxController()

    def send_control_command(self, twist):
        self.publisher_.publish(twist)
        self.get_logger().info(f"Throttle: {twist.linear.z}, Linear X: {twist.linear.x}, Linear Y: {twist.linear.y}, Angular Z: {twist.angular.z}")

def control_loop(node):
    while True:
        left_x, left_y, right_x, right_y = node.controller.read()
        twist = Twist()

        # If any of the values are less than 0.1 of the absolute value, set them to 0
        left_x = left_x if abs(left_x) >= 0.1 else 0
        left_y = left_y if abs(left_y) >= 0.1 else 0
        right_x = right_x if abs(right_x) >= 0.1 else 0
        right_y = right_y if abs(right_y) >= 0.1 else 0

        #Map values.
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
