## @file
#  @brief detection + RViz2 시각화만 실행 (추적 없음, GNSS 불필요).

"""! @brief detection + RViz2 시각화만 실행 (추적 없음, GNSS 불필요). """
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """!
    @brief detection_only.launch.py 를 포함하고 detection_marker_node 와 rviz2(lidar_perception_detection_viz.rviz)를 추가로 구성.
    @return detection(+packet) + detection_marker_node + rviz2 를 담은 LaunchDescription.
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

    viz_config = os.path.join(bringup_share, 'config', 'viz.yaml')
    detection_marker_node = Node(
        package='lidar_perception_viz',
        executable='detection_marker_node',
        name='detection_marker_node',
        output='screen',
        parameters=[viz_config, use_sim_time],
    )

    rviz_dir = os.path.join(bringup_share, 'rviz2')
    # the PointCloud2 display must use the publisher's reliability: reliable for the packet node (input:=packets), best effort for the Ouster driver
    rviz_config = PythonExpression(["'", rviz_dir, "/lidar_perception_detection_viz_reliable.rviz' if ('", LaunchConfiguration('pointcloud_qos'), "' == 'reliable' or ('", LaunchConfiguration('pointcloud_qos'), "' == 'auto' and '", LaunchConfiguration('input'), "' == 'packets')) else '", rviz_dir, "/lidar_perception_detection_viz.rviz'"])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([input_arg, sim_arg, meta_arg, qos_arg, detection_launch, detection_marker_node, rviz_node])
