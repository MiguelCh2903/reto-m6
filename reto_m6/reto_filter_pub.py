#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import random

class RandomCloudPublisher(Node):
    def __init__(self):
        super().__init__('random_cloud_publisher')
        # Publicador de la nube de puntos
        self.publisher_ = self.create_publisher(PointCloud2, 'input_cloud', 10)
        # Timer: cada 2 segundos
        self.timer_ = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        # Generamos puntos aleatorios con un rango más pequeño
        num_points = 500
        points = []
        for _ in range(num_points):
            x = random.uniform(-2.0, 2.0)
            y = random.uniform(-2.0, 2.0)
            z = 0.0
            points.append([x, y, z])

        # Creamos el encabezado
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        # Construimos el mensaje PointCloud2
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)

        # Publicamos la nube
        self.publisher_.publish(cloud_msg)
        self.get_logger().info(f'Publicado {num_points} puntos en input_cloud')

def main(args=None):
    rclpy.init(args=args)
    node = RandomCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
