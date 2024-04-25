#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import threading
from drone.msg import DroneControlData


class FC_Commander(Node):
    def __init__(self):
        super().__init__('fc_command_listener')

        # Define publisher
        self.publisher_status= self.create_publisher(
            DroneControlData,
            '/drone_control_data',
            10
        )
        self.publish_timer = self.create_timer(5, self.status_publisher)

    def test_pub(self):
        msg = DroneControlData()
        msg.vicon_x = float(2)

        # Publish message

        self.publisher_status.publish(msg)

    
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