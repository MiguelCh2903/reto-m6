from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('reto_m6'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'reto_m6',
            'urdf_package_path': PathJoinSubstitution(['reto_robot.urdf'])}.items()
    ))

    return ld