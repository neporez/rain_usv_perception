## @file
#  @brief perception.launch.py(탐지+추적)에 시각화(marker_publisher_node)와 RViz2 까지 더해 실행.

"""! @brief perception.launch.py(탐지+추적)에 시각화(marker_publisher_node)와 RViz2 까지 더해 실행. """
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """!
    @brief perception.launch.py 를 포함하고 marker_publisher_node 와 rviz2(lidar_perception_viz.rviz)를 추가로 구성.
    @return detection + tracking + marker_publisher_node + rviz2 를 담은 LaunchDescription.
    """
    bringup_share = get_package_share_directory('lidar_perception_bringup')
    input_arg = DeclareLaunchArgument('input', default_value='pointcloud', description="'pointcloud' or 'packets'")
    sim_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    meta_arg = DeclareLaunchArgument('ouster_metadata_path', default_value='')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'perception.launch.py')),
        launch_arguments={'input': LaunchConfiguration('input'), 'use_sim_time': LaunchConfiguration('use_sim_time'), 'ouster_metadata_path': LaunchConfiguration('ouster_metadata_path')}.items(),
    )

    viz_config = os.path.join(bringup_share, 'config', 'viz.yaml')
    viz_node = Node(
        package='lidar_perception_viz',
        executable='marker_publisher_node',
        name='marker_publisher_node',
        output='screen',
        parameters=[viz_config, use_sim_time],
    )

    rviz_config = os.path.join(bringup_share, 'rviz2', 'lidar_perception_viz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([input_arg, sim_arg, meta_arg, 
        perception_launch,
        viz_node,
        rviz_node,
    ])