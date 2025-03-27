#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import struct
from sensor_msgs.msg import PointCloud2

class ShapeClassifier(Node):
    def __init__(self):
        super().__init__('shape_classifier')
        self.subscription = self.create_subscription(
            PointCloud2, 'random_shapes', self.listener_callback, 10)
        # Crear una única ventana para mostrar la imagen
        cv2.namedWindow("Clasificación de Figura", cv2.WINDOW_AUTOSIZE)
        self.get_logger().info("ShapeClassifier escuchando /random_shapes")

    def listener_callback(self, msg: PointCloud2) -> None:
        points = self.pointcloud2_to_array(msg)
        if points is None or points.size == 0:
            self.get_logger().warn("No se recibieron puntos.")
            return

        img = self.classify_and_draw(points)
        cv2.imshow("Clasificación de Figura", img)
        cv2.waitKey(1)

    def pointcloud2_to_array(self, msg: PointCloud2) -> np.ndarray:
        if not msg.data:
            return None
        num_points = msg.width * msg.height
        if msg.point_step == 12:
            pts = np.frombuffer(msg.data, dtype=np.float32).reshape(num_points, 3)
        else:
            pts = [struct.unpack_from('fff', msg.data, i * msg.point_step)
                   for i in range(num_points)]
            pts = np.array(pts, dtype=np.float32)
        return pts[:, :2]

    def classify_and_draw(self, points: np.ndarray, img_size: int = 500) -> np.ndarray:
        # Normalizar puntos para ajustarlos a la imagen
        min_xy = np.min(points, axis=0)
        max_xy = np.max(points, axis=0)
        scale = img_size / (np.max(max_xy - min_xy) + 1e-5)
        pts_pixel = ((points - min_xy) * scale).astype(np.int32)
        pts_pixel = np.clip(pts_pixel, 0, img_size - 1)

        # Calcular el convex hull
        hull = cv2.convexHull(pts_pixel)

        # Aproximar el contorno para clasificar la figura
        epsilon = 0.04 * cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, epsilon, True)
        num_vertices = len(approx)
        if num_vertices == 3:
            shape = "Triangule"
        elif num_vertices == 4:
            shape = "Square"
        elif num_vertices == 5:
            shape = "Pentagon"
        else:
            shape = "Circle"

        # Crear imagen en negro y dibujar sobre ella
        img = np.zeros((img_size, img_size, 3), dtype=np.uint8)
        # Dibujar puntos originales (azul)
        for pt in pts_pixel:
            cv2.circle(img, tuple(pt), 2, (255, 0, 0), -1)
        # Dibujar el convex hull (verde)
        cv2.polylines(img, [hull], isClosed=True, color=(0, 255, 0), thickness=2)
        # Mostrar el nombre de la figura en la imagen
        cv2.putText(img, shape, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 255), 2, cv2.LINE_AA)
        return img

def main(args=None):
    rclpy.init(args=args)
    node = ShapeClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
