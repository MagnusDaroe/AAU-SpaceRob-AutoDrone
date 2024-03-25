#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_msg = Twist()

    def send_control_command(self, linear_x, linear_y, angular_z):
        self.control_msg.linear.x = linear_x
        self.control_msg.linear.y = linear_y
        self.control_msg.angular.z = angular_z
        self.publisher_.publish(self.control_msg)


def Manual_Control():
    rclpy.init()
    node = ManualControlNode()

    # Initialize Pygame
    pygame.init()

    # List with values of the keys: ('key', [value increment, index of the control list])
    CONTROL_DICT = {'w': [1, 0], 's': [-1, 0], 'a': [1, 1], 'd': [-1, 1], 'space': [1, 2], 'shift': [-1, 2], 'q':[1,3], 'e':[-1,3]}
    control = [0, 0, 0, 0]

    try:
        # Pygame loop
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    key = pygame.key.name(event.key)
                    if key in CONTROL_DICT:
                        control[CONTROL_DICT[key][1]] += CONTROL_DICT[key][0]
                        print(control)
                        node.send_control_command(control[0], control[1], control[3])
                    elif key == 'escape':
                        running = False

    finally:
        # Shutdown ROS 2 node
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    Manual_Control()
