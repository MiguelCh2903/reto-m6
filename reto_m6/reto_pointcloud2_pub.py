import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', 10)
        # Publicamos a intervalos regulares (cada 0.5 segundos)
        self.timer = self.create_timer(0.5, self.publish_point_cloud)

    def publish_point_cloud(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Asegúrate de que el frame coincide con el que usarás en Rviz

        # Creamos una cuadrícula de puntos en el plano z=0
        points = []
        for x in np.linspace(-1, 1, 10):
            for y in np.linspace(-1, 1, 10):
                z = 0.0
                points.append([x, y, z])
        points = np.array(points, dtype=np.float32)

        # Configuramos la nube de puntos
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 12  # Cada punto: 3 coordenadas (float32 => 4 bytes cada una)
        msg.row_step = msg.point_step * msg.width

        # Definimos los campos de la nube (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Convertimos los puntos a una secuencia de bytes
        msg.data = points.tobytes()

        # Publicamos el mensaje
        self.publisher_.publish(msg)
        self.get_logger().info('Publicado PointCloud2 con {} puntos'.format(msg.width))

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
