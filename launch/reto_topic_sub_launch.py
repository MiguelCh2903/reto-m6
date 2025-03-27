from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='reto_m6',
            executable='listener',
            name='listener',
            output="screen",
            ),
  ])