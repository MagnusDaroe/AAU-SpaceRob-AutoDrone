#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk

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

def on_key_press(event):
    key = event.keysym
    print(key)
    if key in CONTROL_DICT:
        control[CONTROL_DICT[key][1]] += CONTROL_DICT[key][0]
        print(control)
        # Publish control command via ROS 2
        node.send_control_command(float(control[0]), float(control[1]), float(control[3]))
    elif key == 'q':
        # Signal to exit the loop
        root.destroy()

def Manual_Control():
    global root, node, CONTROL_DICT, control
    rclpy.init()
    node = ManualControlNode()

    root = tk.Tk()
    root.title("Manual Control")

    # List with values of the keys: ('key', [value increment, index of the control list])
    CONTROL_DICT = {'w': [1, 0], 's': [-1, 0], 'a': [1, 1], 'd': [-1, 1], 'space': [1, 2], 'Shift_L': [-1, 2], 'q': [1, 3], 'e': [-1, 3]}
    control = [0, 0, 0, 0]

    # Bind key press event to the function
    root.bind('<KeyPress>', on_key_press)

    root.mainloop()

    print("escaped")

    # Shutdown ROS 2 node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    Manual_Control()
