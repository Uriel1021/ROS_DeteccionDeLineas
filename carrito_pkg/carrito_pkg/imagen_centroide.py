import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor_node')
        self.angle_publisher_ = self.create_publisher(Float32, 'processed_angle_centroide', 10)
        self.image_subscription_ = self.create_subscription(
            Image, 'static_image', self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if img is not None:
            # Resto del código de procesamiento de imagen
            img_cortada = self.cortar_imagen(img)
            img_hom = self.homografia_imagen(img_cortada)
            angle = self.detectar_y_obtener_angulo(img_hom)

            # Publicar el ángulo procesado
            angle_msg = Float32()
            angle_msg.data = angle
            self.angle_publisher_.publish(angle_msg)
            self.get_logger().info(f'Publicando el ángulo procesado {angle}')

    def cortar_imagen(self, imagen):
        h, w, _ = np.shape(imagen)
        imgcopy = imagen[int(4 * h / 8):h, int(3 * w / 8):w, :].copy()
        return imgcopy

    def homografia_imagen(self, imagen):
        scale = 4
        img_rs = cv2.resize(imagen, None, fx=1. / scale, fy=1. / scale, interpolation=cv2.INTER_LANCZOS4)
        b, g, r = cv2.split(img_rs)
        img_rs = cv2.merge([r, g, b])
        rows = img_rs.shape[0]
        cols = img_rs.shape[1]
        pts1 = np.float32([[0, 160], [0, rows], [cols, 160], [cols, rows]])
        x = 191
        pts2 = np.float32([[0, 0], [x, rows], [cols, 0], [cols - x, rows]])
        M = cv2.getPerspectiveTransform(pts1, pts2)
        img_hom = cv2.warpPerspective(img_rs, M, (cols, rows))
        return img_hom

    def detectar_y_obtener_angulo(self, imagen):
        gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
        desenfoque = cv2.GaussianBlur(gris, (5, 5), 0)
        bordes = cv2.Canny(desenfoque, 150, 350)

        # Reemplazar la detección de líneas con el algoritmo de centroides
        lines = self.centroid_lines(bordes)
        
        if lines is not None and len(lines) > 0:
            x1, y1, x2, y2 = lines
            angulo_rad = np.arctan2(y2 - y1, x2 - x1)
            angulo_grados = np.degrees(angulo_rad) * (-1)
            return angulo_grados
        else:
            return 0.0

    def centroid_lines(self, img):
        left_centroid = [0, 0]
        right_centroid = [0, 0]
        left_count, right_count = 0, 0

        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            for point in contour:
                x, y = point[0]
                if x < img.shape[1] / 2:  # Left lane
                    left_centroid[0] += x
                    left_centroid[1] += y
                    left_count += 1
                elif x > img.shape[1] / 2:  # Right lane
                    right_centroid[0] += x
                    right_centroid[1] += y
                    right_count += 1

        if left_count > 0:
            left_centroid = np.array(left_centroid) / left_count
        if right_count > 0:
            right_centroid = np.array(right_centroid) / right_count

        return np.array([left_centroid[0], left_centroid[1], right_centroid[0], right_centroid[1]])

def main(args=None):
    rclpy.init(args=args)
    image_processor_node = ImageProcessorNode()
    rclpy.spin(image_processor_node)
    image_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
