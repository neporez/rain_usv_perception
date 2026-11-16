"""Detection + Tracking. input:=pointcloud (기본) | packets, use_sim_time:=true (bag 재생 시)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('lidar_perception_bringup')
    input_arg = DeclareLaunchArgument('input', default_value='pointcloud', description="'pointcloud' or 'packets'")
    sim_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    meta_arg = DeclareLaunchArgument('ouster_metadata_path', default_value='')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'detection_only.launch.py')),
        launch_arguments={'input': LaunchConfiguration('input'), 'use_sim_time': LaunchConfiguration('use_sim_time'), 'ouster_metadata_path': LaunchConfiguration('ouster_metadata_path')}.items(),
    )
    tracking_node = Node(
        package='lidar_perception_tracking', executable='lidar_tracking_node', name='lidar_tracking_node', output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'tracker.yaml'), use_sim_time],
    )
    return LaunchDescription([input_arg, sim_arg, meta_arg, detection_launch, tracking_node])
