import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('lidar_perception_bringup')

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'perception.launch.py')
        )
    )

    viz_config = os.path.join(bringup_share, 'config', 'viz.yaml')
    viz_node = Node(
        package='lidar_perception_viz',
        executable='marker_publisher_node',
        name='marker_publisher_node',
        output='screen',
        parameters=[viz_config],
    )

    rviz_config = os.path.join(bringup_share, 'rviz2', 'lidar_perception_viz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        perception_launch,
        viz_node,
        rviz_node,
    ])