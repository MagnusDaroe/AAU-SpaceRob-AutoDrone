#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from drone.msg import ViconData
import time
import socket





class ViconPublisher(Node):
    def __init__(self):
        super().__init__('vicon_publisher')
        
        self.publisher_ = self.create_publisher(ViconData, '/ViconData', 10)
        self.timer = self.create_timer(1/110, self.publish_message)
            
            
            


    def publish_message(self):
            msg = ViconData()

            self.time = time.time()
            received_data = [10/self.time * 10**9, 8/self.time * 10**9, 1/self.time * 10**9, 2/self.time * 10**9, 5/self.time * 10**9, 7/self.time * 10**9] 
            
            if len(received_data) == 6:
                x, y, z, roll, pitch, yaw = map(float, received_data)
                #(f"Received: {x}, {y}, {z}, {roll}, {pitch}, {yaw}")
                msg.timestamp = time.time()  # Current timestamp
                msg.vicon_x = x  
                msg.vicon_y = y  
                msg.vicon_z = z  
                msg.vicon_roll = roll  
                msg.vicon_pitch = pitch  
                msg.vicon_yaw = yaw
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: %s' % msg)

            else:
                self.get_logger().warning('Publishing: %s' % "Unexpected format")
                self.get_logger().warning('Publishing: %s' % received_data)
                #print(f"Received data has unexpected format: {received_data}")


        

def main(args=None):
    rclpy.init(args=args)
    publisher = ViconPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




