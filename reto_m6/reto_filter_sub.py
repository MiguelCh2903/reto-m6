#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import math

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class CloudFilterNode(Node):
    def __init__(self):
        super().__init__('cloud_filter_node')
        # Suscripción a la nube de puntos original
        self.subscription = self.create_subscription(
            PointCloud2,
            'input_cloud',
            self.cloud_callback,
            10)
        # Publicador de la nube de puntos filtrada (con color)
        self.filtered_pub = self.create_publisher(PointCloud2, 'filtered_cloud', 10)
        # Publicador del marcador que representa la circunferencia del filtro
        self.marker_pub = self.create_publisher(Marker, 'filter_circle', 10)

    def cloud_callback(self, msg: PointCloud2):
        # Generar parámetros aleatorios para el círculo:
        center_x = random.uniform(-0.5, 0.5)
        center_y = random.uniform(-0.5, 0.5)
        radius = random.uniform(0.5, 1.2)  # Se puede ajustar el mínimo si se desea

        self.get_logger().info(f'Nuevo círculo: centro=({center_x:.2f},{center_y:.2f}), radio={radius:.2f}')

        # Leer los puntos del mensaje (se asume que el mensaje tiene campos x, y, z)
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        new_points = []

        for p in points:
            x, y, z = p[0], p[1], p[2]
            # Se filtra solo en x,y (z se mantiene)
            if (x - center_x)**2 + (y - center_y)**2 <= radius**2:
                # Punto filtrado: asignar color rojo
                rgb = self.rgb_to_float(255, 0, 0)
            else:
                # Punto sin filtrar: asignar color blanco
                rgb = self.rgb_to_float(255, 255, 255)
            new_points.append([x, y, z, rgb])

        # Definir la cabecera y los campos (agregando el campo 'rgb')
        header = msg.header
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # Crear y publicar la nueva nube de puntos
        filtered_cloud = point_cloud2.create_cloud(header, fields, new_points)
        self.filtered_pub.publish(filtered_cloud)

        # Crear y publicar el marcador (línea en forma de círculo)
        marker = Marker()
        marker.header = header
        marker.ns = "filter_circle"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # Grosor de la línea
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Generar puntos para la circunferencia
        num_segments = 50
        marker.points = []
        for i in range(num_segments + 1):
            angle = 2 * math.pi * i / num_segments
            pt = Point()
            pt.x = center_x + radius * math.cos(angle)
            pt.y = center_y + radius * math.sin(angle)
            pt.z = 0.0  # La circunferencia se dibuja en el plano xy
            marker.points.append(pt)
        self.marker_pub.publish(marker)

    def rgb_to_float(self, r: int, g: int, b: int) -> float:
        """
        Convierte componentes RGB a un float codificado.
        Se empaquetan los valores en un entero de 32 bits y luego se reinterpretan como float.
        """
        # Empaquetado: cada componente ocupa 8 bits: 0x00RRGGBB
        rgb_int = (r << 16) | (g << 8) | b
        # Se utiliza la conversión de int a float de acuerdo a la representación en ROS
        import struct
        return struct.unpack('f', struct.pack('I', rgb_int))[0]

def main(args=None):
    rclpy.init(args=args)
    node = CloudFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
