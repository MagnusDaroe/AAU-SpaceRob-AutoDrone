#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from drone.msg import DroneStatus
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(DroneStatus, 'drone_status', 10)
        self.timer = self.create_timer(1, self.publish_message)
        self.mode = 1


    def publish_message(self):
        msg = DroneStatus()
        msg.timestamp = time.time()  # Current timestamp
        msg.battery_ok = 1  # Example battery_ok value
        msg.battery_percentage = 80.0  # Example battery_percentage value
        msg.fc_connection = 1  # Example fc_connection value
        msg.armed = 1  # Example armed value

        # Toggle mode value between 0 and 1
        if self.mode == 1:
            msg.mode = 0
            self.mode = 0  # Update mode attribute for next iteration
        else:
            msg.mode = 1
            self.mode = 1  # Update mode attribute for next iteration
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: %s' % msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = TestPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
