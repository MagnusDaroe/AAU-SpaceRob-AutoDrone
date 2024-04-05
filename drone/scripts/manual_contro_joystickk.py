import rclpy
from rclpy.node import Node
import threading
import pygame
import numpy as np
from drone.msg import DroneCommand

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control')
        self.publisher_ = self.create_publisher(DroneCommand, '/cmd_fc', 10)
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def send_control_command(self, DroneCommand):
        self.publisher_.publish(DroneCommand)
        print(f"Roll={DroneCommand.cmd_roll}, Pitch={DroneCommand.cmd_pitch}, Thrust={DroneCommand.cmd_thrust}, Yaw={DroneCommand.cmd_yaw}")

def control_loop(node):
    while True:
        pygame.event.pump()  # Process event queue
        ax0 = node.controller.get_axis(0)  # Thrust
        ax3 = node.controller.get_axis(3)  # Yaw
        ax2 = node.controller.get_axis(2)  # Pitch
        ax1 = node.controller.get_axis(1)  # Roll

        print(f"Axis 0: {ax0}, Axis 3: {ax3}, Axis 2: {ax2}, Axis 1: {ax1}")

        but0 = node.controller.get_button(4)  # SG 3-way switch
        but1 = node.controller.get_button(5)  # SC 3-way switch
        but2 = node.controller.get_button(6)  # Right analog button
        but3 = node.controller.get_button(7)  # SF 2-way switch

        print(f"Button 0: {but0}, Button 1: {but1}, Button 2: {but2}, Button 3: {but3}\n")

        Drone_cmd = DroneCommand()

        # Map values.
        Drone_cmd.cmd_roll = float(np.interp(ax1, (-1, 1), (-1000, 1000)))  # Roll
        Drone_cmd.cmd_pitch = float(np.interp(ax2, (-1, 1), (-1000, 1000)))  # Pitch
        Drone_cmd.cmd_thrust = float(np.interp(ax0, (-1, 1), (1000, -1000)))  # Throttle
        Drone_cmd.cmd_yaw = float(np.interp(ax3, (-1, 1), (-1000, 1000)))  # Yaw

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
