import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.image_publisher_ = self.create_publisher(Image, 'static_image', 10)
        timer_period = 1.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.image_path = "/home/uriel/Desktop/imagen2.jpg"
        self.bridge = CvBridge()

    def timer_callback(self):
        img = cv2.imread(self.image_path)

        if img is not None:
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.image_publisher_.publish(img_msg)
            self.get_logger().info('Publicando la imagen')
        else:
            self.get_logger().info(f'Error we {self.image_path}')

def main(args=None):
    rclpy.init(args=args)
    image_publisher_node = ImagePublisherNode()
    rclpy.spin(image_publisher_node)
    image_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
