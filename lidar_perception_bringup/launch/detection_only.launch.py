import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lidar_perception_bringup'),
        'config',
        'detection.yaml'
    )

    return LaunchDescription([
        Node(
            package='lidar_perception_detection',
            executable='lidar_detection_node',
            name='lidar_detection_node',
            output='screen',
            parameters=[config],
        )
    ])