import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class AngleSubscriberNode(Node):
    def __init__(self):
        super().__init__('angle_subscriber_node')
        
        # Primera suscripción a 'processed_angle'
        self.angle_subscription = self.create_subscription(
            Float32, 'processed_angle', self.angle_callback, 10)

        # Segunda suscripción a 'processed_angle_centroide'
        self.angle_centroide_subscription = self.create_subscription(
            Float32, 'processed_angle_centroide', self.angle_centroide_callback, 10)

    def angle_callback(self, msg):
        angle = msg.data
        self.process_angle(angle, "hough")

    def angle_centroide_callback(self, msg):
        angle_centroide = msg.data
        self.process_angle(angle_centroide, "centroide")

    def process_angle(self, angle, source):
        average_angle = angle / 2.0

        if average_angle < 90:
            self.get_logger().info(f'Ángulo promedio de {source}: {average_angle:.2f} grados,  izquierda')
        elif average_angle == 90:
            self.get_logger().info(f'Ángulo promedio de {source}: {average_angle:.2f} grados, recto')
        else:
            self.get_logger().info(f'Ángulo promedio de {source}: {average_angle:.2f} grados, derecha')

def main(args=None):
    rclpy.init(args=args)
    angle_subscriber_node = AngleSubscriberNode()
    rclpy.spin(angle_subscriber_node)
    angle_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

