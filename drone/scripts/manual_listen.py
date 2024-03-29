#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ManualControlListener(Node):
    def __init__(self):
        super().__init__('manual_control_listener')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        angular_z = msg.angular.z

        # Implement control logic here based on received Twist message
        # For now, we'll just print the received values
        print(f"Received control command: linear_x={linear_x}, linear_y={linear_y}, linear_z={linear_z}, angular_z={angular_z}")


def main(args=None):
    rclpy.init(args=args)
    listener = ManualControlListener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
