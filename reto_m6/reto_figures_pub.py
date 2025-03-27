#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import random
import math

from sensor_msgs.msg import PointCloud2, PointField

class RandomShapesPublisher(Node):
    def __init__(self):
        super().__init__('random_shapes_publisher')

        # Publicador de PointCloud2
        self.publisher_ = self.create_publisher(PointCloud2, 'random_shapes', 10)

        # Lista de figuras posibles
        self.shapes = ['circle', 'triangle', 'square', 'pentagon']
        self.last_shape = None

        # Timer para publicar cada cierto intervalo (ej: 2.0 seg)
        self.timer_period = 2.0
        self.timer = self.create_timer(self.timer_period, self.publish_random_shape)

        self.get_logger().info('RandomShapesPublisher iniciado.')

    def publish_random_shape(self):
        """Genera una figura aleatoria y publica la nube de puntos."""
        shape = random.choice(self.shapes)
        while shape == self.last_shape:
            shape = random.choice(self.shapes)
        self.last_shape = shape
        self.get_logger().info(f'Generando forma: {shape}')

        # Generamos puntos con ruido gaussiano
        points = self.generate_points(shape=shape, n_points=100, noise_std=0.02)

        # Convertimos los puntos en un PointCloud2
        cloud_msg = self.points_to_pointcloud2(points, frame_id='map')
        
        # Publicamos el mensaje
        self.publisher_.publish(cloud_msg)
        self.get_logger().info(f'Publicado PointCloud2 con {len(points)} puntos ({shape}).')

    def generate_points(self, shape='circle', n_points=100, noise_std=0.01):
        """
        Genera n_points en la periferia de la figura especificada.
        Agrega ruido gaussiano con desviación estándar noise_std.
        Retorna un array Nx3 (x, y, z=0).
        """

        # Inicializamos un array (x, y, z)
        points = np.zeros((n_points, 3), dtype=np.float32)

        if shape == 'circle':
            # Radio de ~1
            radius = 1.0
            angles = np.linspace(0, 2*math.pi, n_points, endpoint=False)
            x = radius * np.cos(angles)
            y = radius * np.sin(angles)
            points[:, 0] = x
            points[:, 1] = y

        elif shape == 'triangle':
            # Triángulo equilátero con lado ~2
            # Para generarlo, definimos los 3 vértices y repartimos puntos en sus lados.
            vertices = np.array([
                [0.0, 1.15],          # V1
                [1.0, -0.577],        # V2
                [-1.0, -0.577],       # V3
            ], dtype=np.float32)
            points = self.generate_polygon_points(vertices, n_points)

        elif shape == 'square':
            # Cuadrado con lado ~2 (centro en (0,0))
            # Vértices en (-1, -1), (1, -1), (1, 1), (-1, 1)
            vertices = np.array([
                [-1.0, -1.0],
                [ 1.0, -1.0],
                [ 1.0,  1.0],
                [-1.0,  1.0]
            ], dtype=np.float32)
            points = self.generate_polygon_points(vertices, n_points)

        elif shape == 'pentagon':
            # Pentágono regular circunscrito en radio ~1
            # Generamos 5 vértices regularmente espaciados
            num_vertices = 5
            angles = np.linspace(0, 2*math.pi, num_vertices, endpoint=False)
            radius = 1.0
            vertices = np.array([
                [radius * math.cos(a), radius * math.sin(a)] for a in angles
            ], dtype=np.float32)
            points = self.generate_polygon_points(vertices, n_points)

        # Agregamos ruido gaussiano (en x, y)
        noise = np.random.normal(loc=0.0, scale=noise_std, size=(n_points, 2))
        points[:, 0:2] += noise

        # z=0 en todos los puntos (ya está inicializado en cero)
        return points

    def generate_polygon_points(self, vertices, n_points):
        """
        Dado un conjunto de vértices que definen un polígono,
        genera n_points distribuidos uniformemente en su perímetro.
        Retorna un array Nx3.
        """
        # Cerramos el polígono (añadimos el primer vértice al final)
        closed_vertices = np.vstack([vertices, vertices[0]])

        # Calculamos longitudes de cada lado
        side_lengths = []
        for i in range(len(closed_vertices) - 1):
            p1 = closed_vertices[i]
            p2 = closed_vertices[i+1]
            side_lengths.append(np.linalg.norm(p2 - p1))

        # Perímetro total
        perimeter = sum(side_lengths)

        # Distancia promedio entre puntos
        dist_step = perimeter / n_points

        # Generamos los puntos
        points = []
        current_side = 0
        current_pos = 0.0  # Distancia recorrida en el lado actual

        for i in range(n_points):
            # Si estamos al final del lado, pasamos al siguiente
            while current_side < len(side_lengths) and current_pos > side_lengths[current_side]:
                current_pos -= side_lengths[current_side]
                current_side += 1
                if current_side >= len(side_lengths):
                    # Evitar IndexError si llegamos al final
                    current_side = len(side_lengths) - 1
                    break

            # Interpolamos la posición en el lado actual
            p1 = closed_vertices[current_side]
            p2 = closed_vertices[current_side + 1]
            side_len = side_lengths[current_side]

            t = current_pos / side_len  # factor de interpolación [0,1]
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            points.append([x, y, 0.0])

            current_pos += dist_step

        return np.array(points, dtype=np.float32)

    def points_to_pointcloud2(self, points, frame_id='map'):
        """
        Convierte un array de Nx3 (float32) en un mensaje PointCloud2.
        """
        # Crear la estructura base
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = frame_id

        cloud_msg.height = 1
        cloud_msg.width = points.shape[0]
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 floats de 4 bytes cada uno (x, y, z)
        cloud_msg.row_step = cloud_msg.point_step * points.shape[0]

        # Definir los campos (x, y, z)
        cloud_msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1)
        ]

        # Convertir el array de floats a bytes
        cloud_msg.data = points.tobytes()

        return cloud_msg

def main(args=None):
    rclpy.init(args=args)
    node = RandomShapesPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
