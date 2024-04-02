import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.timer_ = self.create_timer(1.0 / 30, self.publish_image)  # Capture at 30 fps
        self.bridge_ = CvBridge()

    def publish_image(self):
        cap = cv2.VideoCapture(0)  # Open the default camera (change index if needed)
        ret, frame = cap.read()
        if ret:
            image_msg = self.bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_msg)
        cap.release()

def main():
    rclpy.init()
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
