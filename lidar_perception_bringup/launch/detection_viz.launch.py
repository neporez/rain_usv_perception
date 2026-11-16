"""Detection + RViz2 시각화만 실행 (추적 없음, GNSS 불필요)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('lidar_perception_bringup')

    detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'detection_only.launch.py'))
    )

    viz_config = os.path.join(bringup_share, 'config', 'viz.yaml')
    detection_marker_node = Node(
        package='lidar_perception_viz',
        executable='detection_marker_node',
        name='detection_marker_node',
        output='screen',
        parameters=[viz_config],
    )

    rviz_config = os.path.join(bringup_share, 'rviz2', 'lidar_perception_detection_viz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([detection_launch, detection_marker_node, rviz_node])
