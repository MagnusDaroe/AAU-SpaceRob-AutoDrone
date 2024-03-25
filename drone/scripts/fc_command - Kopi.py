#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import keyboard

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

    # List with values of the keys: ('key', [value increment, index of the control list])
    CONTROL_DICT = {'w': [1, 0], 's': [-1, 0], 'a': [1, 1], 'd': [-1, 1], 'space': [1, 2], 'shift': [-1, 2], 'q':[1,3], 'e':[-1,3]}
    control = [0, 0, 0, 0]

    # Function to handle key press
    def on_key_press(event):
        nonlocal control
        # Check if the key pressed is in the dictionary
        if event.name in CONTROL_DICT:
            # Map the input to a value increment/decrement in the control list
            control[CONTROL_DICT[event.name][1]] += CONTROL_DICT[event.name][0]
            print(control)
            # Publish control command via ROS 2
            node.send_control_command(control[0], control[1], control[3])
        # Check if the escape key is pressed
        elif event.name == 'esc':
            # Signal to exit the loop
            return False
        
        return True

    # Subscribe to keyboard events
    keyboard.on_press(on_key_press)
    
    # Loop until escape key is pressed
    while True:
        if not keyboard.is_pressed('esc'):
            continue
        else:
            break
    
    # Unsubscribe from keyboard events
    keyboard.unhook_all()
    print("escaped")

    # Shutdown ROS 2 node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    Manual_Control()
