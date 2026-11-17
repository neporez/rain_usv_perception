## @file
#  @brief detection_node 만 실행하는 launch 파일 (input:=packets 이면 ouster_packet_node 추가).

"""! @brief detection_node 만 실행하는 launch 파일 (input:=packets 이면 ouster_packet_node 추가). """
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """!
    @brief detection.yaml 로 detection_node 를, input=packets 이면 ouster_packets.yaml 로 ouster_packet_node 를 함께 구성. 인자: input, use_sim_time, ouster_metadata_path, pointcloud_qos.
    @return detection_node(+ ouster_packet_node)를 담은 LaunchDescription.
    """
    share = get_package_share_directory('lidar_perception_bringup')
    input_arg = DeclareLaunchArgument('input', default_value='pointcloud', description="'pointcloud' or 'packets'")
    sim_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    meta_arg = DeclareLaunchArgument('ouster_metadata_path', default_value='', description="Ouster metadata JSON; '' = the packaged lidar_perception_detection/config/ouster_metadata.json (the latched /dev/lidar/metadata topic overrides it)")
    qos_arg = DeclareLaunchArgument('pointcloud_qos', default_value='auto', description="point cloud subscription QoS: 'auto' = reliable with input:=packets (packet node publishes reliable), best_effort otherwise (Ouster driver default); or 'reliable' / 'best_effort'")
    pointcloud_qos = PythonExpression(["'reliable' if ('", LaunchConfiguration('pointcloud_qos'), "' == 'reliable' or ('", LaunchConfiguration('pointcloud_qos'), "' == 'auto' and '", LaunchConfiguration('input'), "' == 'packets')) else 'best_effort'"])
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    packet_node = Node(
        package='lidar_perception_detection', executable='ouster_packet_node', name='ouster_packet_node', output='screen',
        parameters=[os.path.join(share, 'config', 'ouster_packets.yaml'), use_sim_time, {'metadata_path': LaunchConfiguration('ouster_metadata_path')}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('input'), "' == 'packets'"])),
    )
    detection_node = Node(
        package='lidar_perception_detection', executable='lidar_detection_node', name='lidar_detection_node', output='screen',
        parameters=[os.path.join(share, 'config', 'detection.yaml'), use_sim_time, {'pointcloud_qos_reliability': pointcloud_qos}],
    )
    return LaunchDescription([input_arg, sim_arg, meta_arg, qos_arg, packet_node, detection_node])
