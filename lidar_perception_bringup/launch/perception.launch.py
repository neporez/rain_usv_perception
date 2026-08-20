import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('lidar_perception_bringup')

    detection_config = os.path.join(bringup_share, 'config', 'detection.yaml')
    tracking_config = os.path.join(bringup_share, 'config', 'tracking.yaml')

    return LaunchDescription([
        Node(
            package='lidar_perception_detection',
            executable='lidar_detection_node',
            name='lidar_detection_node',
            output='screen',
            parameters=[detection_config],
        ),
        Node(
            package='lidar_perception_tracking',
            executable='lidar_tracking_node',
            name='lidar_tracking_node',
            output='screen',
            parameters=[tracking_config],
        ),
    ])