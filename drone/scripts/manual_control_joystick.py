#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import pygame
import numpy as np
from drone.msg import DroneCommand
import time

# Mode dictionary
mode_dict = {0: 'Manual_Control', 1: 'Autonomous', 2: 'Undefined'}

class JoystickControlNode(Node):
    def __init__(self):
        time.time() 
        super().__init__('joystick_control')
        self.publisher_ = self.create_publisher(DroneCommand, '/cmd_fc', 10)
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def send_control_command(self, DroneCommand):
        rate = self.create_rate(100)
        self.publisher_.publish(DroneCommand)
        print(f"Armed={DroneCommand.cmd_arm}, Estop={DroneCommand.cmd_estop}, mode={mode_dict[DroneCommand.cmd_mode]}, Timestamp={DroneCommand.timestamp},Roll={DroneCommand.cmd_roll}, Pitch={DroneCommand.cmd_pitch}, Thrust={DroneCommand.cmd_thrust}, Yaw={DroneCommand.cmd_yaw}")
        rate.sleep()

def control_loop(node):
    
    while True:
        pygame.event.pump()  # Process event queue
        ax0 = node.controller.get_axis(0)  # Thrust
        ax3 = node.controller.get_axis(3)  # Yaw
        ax2 = node.controller.get_axis(2)  # Pitch
        ax1 = node.controller.get_axis(1)  # Roll

        mode = int(2) if node.controller.get_axis(5) > 0 else int(node.controller.get_axis(5))+1  # SC 3-way switch
        arm = int(node.controller.get_axis(6))+1  # SF 3-way switch 
        estop = int(node.controller.get_axis(4))+1  # SG 3-way switch 0 is normal operation, above 0 is Emergency stop


        Drone_cmd = DroneCommand()

        Drone_cmd.timestamp = float(time.time())
        Drone_cmd.cmd_mode = mode
        Drone_cmd.cmd_arm = True if arm == 1 else False
        Drone_cmd.cmd_estop = True if estop > 0 else False

        if Drone_cmd.cmd_mode == 1:
            time.sleep(0.2)
        else:
            # Map values.   
            Drone_cmd.cmd_roll = float(np.interp(ax2, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # Roll
            Drone_cmd.cmd_pitch = float(np.interp(ax1, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))   # Pitch
            Drone_cmd.cmd_thrust = float(np.interp(ax0, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # Throttle
            Drone_cmd.cmd_yaw = float(np.interp(ax3, (-1, -0.1, 0.1, 1), (-1000, 0, 0, 1000)))  # Yaw
        node.send_control_command(Drone_cmd)
        

def main(args=None):
    rclpy.init(args=args)
    joystick_control_node = JoystickControlNode()
    control_thread = threading.Thread(target=control_loop, args=(joystick_control_node,))
    control_thread.start()
    rclpy.spin(joystick_control_node)
    control_thread.join()
    joystick_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
