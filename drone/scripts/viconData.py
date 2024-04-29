#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from drone.msg import DroneControlData
import time
import socket





class ViconPublisher(Node):
    def __init__(self):
        super().__init__('vicon_publisher')
        
        self.publisher_ = self.create_publisher(DroneControlData, 'DroneControlData', 10)
        self.timer = self.create_timer(0.1, self.publish_message)
        self.mode = 1
        self.HOST = '192.168.1.144'  # Listen on all network interfaces
        self.PORT = 12345      # Choose a port to listen on
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind((self.HOST, self.PORT))
            server_socket.listen(1)
            print(f"Server listening on {self.HOST}:{self.PORT}")

            self.conn, self.addr = server_socket.accept()
            print(f"Connection from {self.addr}")
            
            


    def publish_message(self):
            msg = DroneControlData()
            self.data = self.conn.recv(1024)
            if not self.data:
                return
            
            received_data = self.data.decode().split(',')

            if len(received_data) == 6:
                x, y, z, roll, pitch, yaw = map(float, received_data)
                #(f"Received: {x}, {y}, {z}, {roll}, {pitch}, {yaw}")
                #msg.timestamp = time.time()  # Current timestamp
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




