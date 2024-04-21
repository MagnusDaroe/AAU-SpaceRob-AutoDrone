#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from drone.srv import Clock

class Clock(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Clock, '/clock', self.get_server_time)

    def get_server_time(self, request, response):
        response.ServerTime = time.time()
        return response

def main(args=None):
    rclpy.init(args=args)

    clock = Clock()
    rclpy.spin(clock)

    rclpy.shutdown()

if __name__ == '__main__':
    main()