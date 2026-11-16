## @page interfaces_page Message Interfaces
#  @brief Custom ROS2 message definitions (lidar_perception_interfaces / detect_msgs packages).
#
#  파이프라인 각 단계가 주고받는 메시지 스키마다. 순수 `.msg` 정의 패키지라
#  Python/C++ 소스는 없고, 아래는 원본 `.msg` 파일 내용(필드별 ROS 주석 포함)을
#  그대로 가져온 것이다.
#
#  @par Object3D
#  단일 탐지 객체. detection_node가 발행하는 Object3DArray의 원소. 9자유도(yaw, roll, pitch) 상자.
#  @verbinclude Object3D.msg
#
#  @par Object3DArray
#  detection_node의 최종 발행 메시지.
#  @verbinclude Object3DArray.msg
#
#  @par TrackedObject3D
#  추적 ID/속도가 붙은 단일 객체. TrackedObject3DArray의 원소.
#  @verbinclude TrackedObject3D.msg
#
#  @par TrackedObject3DArray
#  marker_publisher_node가 구독하는 시각화용 메시지.
#  @verbinclude TrackedObject3DArray.msg
#
#  @par ObjectsDetected
#  detect_msgs 포맷. 타 시스템 호환용 tracking_node의 최종 출력.
#  @verbinclude ObjectsDetected.msg
#
#  @par InfoDetectedObject
#  ObjectsDetected 안에 담기는 객체 1개. 필드명이 픽셀 기준이지만
#  이 프로젝트에서는 실제로 로컬 좌표(m)가 들어간다.
#  @verbinclude InfoDetectedObject.msg

## @file
#  @brief LiDAR 객체 탐지 ROS2 노드 진입점.

"""! @brief LiDAR 객체 탐지 ROS2 노드 진입점. """
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from lidar_perception_interfaces.msg import Object3D, Object3DArray
from lidar_perception_detection.detection_utils import resolve_qos, parse_pointcloud2
from lidar_perception_detection.model_adapter import ModelInferencer

class DetectionNode(Node):
    """!
    @brief LiDAR 포인트클라우드 1프레임에 대한 3D 객체 탐지 노드.

    `pointcloud_topic`(기본 `/dev/lidar/points`)을 구독해 lidar_callback()에서
    VoxelNeXt(OpenPCDet) 모델 추론을 수행하고, score_threshold 이상인 결과만
    Object3DArray로 발행한다. 발행되는 좌표는 LiDAR 센서 로컬 좌표계(os_lidar) 기준이며,
    global 좌표 변환과 프레임 간 추적은 tracking_node가 담당한다. 입력이 Ouster 원시
    패킷이면 ouster_packet_node 가 먼저 PointCloud2 로 변환해 같은 토픽으로 넣어 준다.

    주요 구성:
    - __init__(): 파라미터 선언, 모델 로드(ModelInferencer), 구독/발행자 생성.
    - lidar_callback(): 탐지 메인 처리 (포인트클라우드 파싱 → 추론 → 메시지 변환/발행).
    """

    def __init__(self):
        """!
        @brief 파라미터 선언, 모델 로드, 구독/발행자 생성.
        """
        super().__init__("lidar_detection_node")

        self.declare_parameter("pointcloud_topic", "/dev/lidar/points")
        self.declare_parameter("pointcloud_qos_reliability", "best_effort")
        self.declare_parameter("model_config_path", "cfgs/kriso_models/kriso_voxelnext_deploy.yaml")
        self.declare_parameter("ckpt_path", "cfgs/ckpt/voxelnext_kriso_real_epoch_80.pth")
        self.declare_parameter("score_threshold", 0.3)
        self.declare_parameter("output_topic", "/detection/objects")

        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        qos_reliability = self.get_parameter("pointcloud_qos_reliability").value
        model_config_path = self.get_parameter("model_config_path").value
        ckpt_path = self.get_parameter("ckpt_path").value
        ## 탐지 결과 채택 최소 신뢰도 (기본 0.3)
        self.score_threshold = self.get_parameter("score_threshold").value
        output_topic = self.get_parameter("output_topic").value

        self.get_logger().info("Loading model...")
        ## VoxelNeXt(OpenPCDet) 추론기
        self.inferencer = ModelInferencer(
            cfg_path=model_config_path,
            ckpt_path=ckpt_path,
            score_thresh=self.score_threshold
        )
        self.get_logger().info("Model loaded.")

        qos = resolve_qos(qos_reliability, depth=1)
        ## LiDAR PointCloud2 구독자
        self.sub = self.create_subscription(
            PointCloud2, pointcloud_topic, self.lidar_callback, qos
        )
        ## 탐지 결과(Object3DArray) 발행자
        self.pub = self.create_publisher(Object3DArray, output_topic, 10)

    def lidar_callback(self, msg: PointCloud2):
        """!
        @brief 포인트클라우드 1프레임에 대한 객체 탐지 메인 콜백.

        1. **포인트클라우드 파싱**: PointCloud2 → (N,4) float32 numpy [x, y, z, intensity] 배열로 변환한다 (parse_pointcloud2). 모델은 이 중 x, y, z 만 사용한다.
        2. **모델 추론**: ModelInferencer.predict()로 9자유도 3D bbox 후보를 얻는다. 생성 시 고정된 score_threshold 이상인 결과만 반환된다.
        3. **메시지 변환 및 발행**: 각 탐지 결과를 Object3D(center, size, yaw, roll, pitch, class, score)로 채워 Object3DArray로 발행한다. 헤더(timestamp/frame_id)는 원본 LiDAR 프레임 것을 그대로 유지해, tracking_node가 GNSS pose와 시간을 맞출 수 있게 한다.

        @param msg 원본 LiDAR PointCloud2 메시지.
        """

        points = parse_pointcloud2(msg)

        detections = self.inferencer.predict(points)

        # 3. Object3DArray로 변환 후 발행
        out_msg = Object3DArray()
        out_msg.header = msg.header  # 원본 LiDAR 프레임 timestamp/frame_id 유지

        for det in detections:
            obj = Object3D()
            obj.center.x, obj.center.y, obj.center.z = det["center"]
            obj.size.x, obj.size.y, obj.size.z = det["size"]
            obj.yaw = det["yaw"]
            obj.roll = det["roll"]
            obj.pitch = det["pitch"]
            obj.class_id = det["class_id"]
            obj.class_name = det["class_name"]
            obj.score = det["score"]
            out_msg.objects.append(obj)

        self.pub.publish(out_msg)


def main(args=None):
    """!
    @brief ROS2 노드 실행 진입점 (rclpy 초기화 → spin → 종료).
    @param args rclpy.init()에 전달할 커맨드라인 인자.
    """
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()