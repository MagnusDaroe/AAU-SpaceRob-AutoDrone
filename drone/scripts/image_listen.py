import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraListener(Node):
    def __init__(self):
        super().__init__('camera_listener')
        self.subscription_ = self.create_subscription(
            Image,
            'camera_publisher',  # Topic name (should match the publisher)
            self.image_callback,
            10  # QoS profile depth
        )
        self.bridge_ = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the image
            cv2.imshow('Camera Image', cv_image)
            cv2.waitKey(1)  # Wait for a key press (1 ms delay)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main():
    rclpy.init()
    camera_listener = CameraListener()
    rclpy.spin(camera_listener)
    camera_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
