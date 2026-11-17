## @file
#  @brief detection + tracking 을 함께 실행하는 launch 파일 (시각화 제외).

"""! @brief detection + tracking 을 함께 실행하는 launch 파일 (시각화 제외). """
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """!
    @brief detection_only.launch.py 를 포함하고 tracker.yaml 로 tracking_node 를 추가. 인자(input, use_sim_time, ouster_metadata_path)는 그대로 전달.
    @return detection(+packet) + tracking 노드를 담은 LaunchDescription.
    """
    bringup_share = get_package_share_directory('lidar_perception_bringup')
    input_arg = DeclareLaunchArgument('input', default_value='pointcloud', description="'pointcloud' or 'packets'")
    sim_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    meta_arg = DeclareLaunchArgument('ouster_metadata_path', default_value='')
    qos_arg = DeclareLaunchArgument('pointcloud_qos', default_value='auto')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'detection_only.launch.py')),
        launch_arguments={'input': LaunchConfiguration('input'), 'use_sim_time': LaunchConfiguration('use_sim_time'), 'ouster_metadata_path': LaunchConfiguration('ouster_metadata_path'), 'pointcloud_qos': LaunchConfiguration('pointcloud_qos')}.items(),
    )
    tracking_node = Node(
        package='lidar_perception_tracking', executable='lidar_tracking_node', name='lidar_tracking_node', output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'tracker.yaml'), use_sim_time],
    )
    return LaunchDescription([input_arg, sim_arg, meta_arg, qos_arg, detection_launch, tracking_node])
