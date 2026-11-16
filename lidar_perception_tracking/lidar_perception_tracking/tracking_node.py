## @file
#  @brief 다중 객체 추적 ROS2 노드 진입점.

"""! @brief 다중 객체 추적 ROS2 노드 진입점. """
import numpy as np
import rclpy
from rclpy.node import Node

from gnss_msgs.msg import Pose
from lidar_perception_interfaces.msg import (
    Object3DArray,
    TrackedObject3D,
    TrackedObject3DArray,
)
from detect_msgs.msg import ObjectsDetected, InfoDetectedObject
from lidar_perception_tracking.pose_cache import PoseCache
from lidar_perception_tracking.tracker_adapter import TrackerAdapter


class TrackingNode(Node):
    """!
    @brief Object3DArray 탐지 결과와 GNSS/INS pose를 받아 다중 객체 추적을 수행하는 노드.

    detection_node의 출력(Object3DArray, 10Hz)과 자선 pose(gnss_msgs/Pose; hemisphere GNSS 10Hz
    또는 Spatial FOG INS 100Hz)를 각각 구독하다가, 탐지가 들어올 때마다 그 LiDAR 프레임 시각의
    pose를 PoseCache 버퍼에서 **보간**해 로컬→글로벌 변환 행렬을 만들고, TrackerAdapter로
    프레임 간 ID 추적을 수행한다. 결과는 두 가지 포맷으로 동시에 발행한다.

    주요 구성:
    - on_gnss(): pose 수신 시 PoseCache 버퍼 갱신.
    - on_detections(): 탐지 수신 시 추적 수행 및 결과 발행 (메인 처리).
    - object3d_array_to_boxes(): Object3DArray → (N,7) numpy 배열 변환.
    """

    def __init__(self):
        """!
        @brief 파라미터 선언, TrackerAdapter/PoseCache 생성, 구독/발행자 생성.
        """
        super().__init__("tracking_node")

        self.declare_parameter("detection_topic", "/detection/objects")
        self.declare_parameter("output_topic", "/alg1/dl_pst_detection")
        self.declare_parameter("viz_output_topic", "/tracking/objects_viz")
        # pose source: 10 Hz hemisphere GNSS (= /system/gnss/pose) by default; /dev/spatial_fog_dual/pose (100 Hz INS)
        # also works. The 1 Hz /digital_bridge/gnss/pose is NOT recommended (would need extrapolation).
        self.declare_parameter("gnss_topic", "/dev/gnss/hemisphere/pose")
        self.declare_parameter("pose_time_source", "header")       # 'header' (host clock, same as the LiDAR) | 'gnss_time'
        self.declare_parameter("pose_max_age_sec", 0.5)            # LiDAR stamp outside the pose buffer by more than this -> frame skipped
        self.declare_parameter("pose_buffer_sec", 5.0)
        self.declare_parameter("pose_time_offset_sec", 0.0)        # added to the LiDAR stamp before the lookup
        self.declare_parameter("heading_convention", "ned")        # 'ned': Pose.yaw = deg clockwise from north (message spec) | 'enu'
        self.declare_parameter("tracker_config_path", "tracker/config/multi_object_tracker.yaml")

        detection_topic = self.get_parameter("detection_topic").value
        output_topic = self.get_parameter("output_topic").value
        viz_output_topic = self.get_parameter("viz_output_topic").value
        gnss_topic = self.get_parameter("gnss_topic").value
        tracker_config_path = self.get_parameter("tracker_config_path").value

        ## ROS ↔ Tracker3D 어댑터
        self.tracker = TrackerAdapter(cfg_path=tracker_config_path)
        ## 자선 pose 버퍼 (LiDAR 시각으로 보간)
        self.pose_cache = PoseCache(
            max_age_sec=self.get_parameter("pose_max_age_sec").value,
            buffer_sec=self.get_parameter("pose_buffer_sec").value,
            time_source=self.get_parameter("pose_time_source").value,
            heading_convention=self.get_parameter("heading_convention").value,
            time_offset_sec=self.get_parameter("pose_time_offset_sec").value,
        )
        self.get_logger().info(f"pose source {gnss_topic} (time: {self.pose_cache.time_source}, heading: {self.pose_cache.heading_convention}, max age {self.pose_cache.max_age_sec}s)")

        ## GNSS/INS pose 구독자
        self.gnss_sub = self.create_subscription(
            Pose, gnss_topic, self.on_gnss, 100       # 100 Hz INS 소스도 밀리지 않도록 깊이 100
        )
        ## 탐지 결과(Object3DArray) 구독자
        self.detection_sub = self.create_subscription(
            Object3DArray, detection_topic, self.on_detections, 10
        )

        ## 트래커 출력 발행자 (detect_msgs/ObjectsDetected 포맷, 타 시스템 호환용)
        self.pub = self.create_publisher(ObjectsDetected, output_topic, 10)
        ## 시각화용 발행자 (TrackedObject3DArray 포맷)
        self.pub_viz = self.create_publisher(TrackedObject3DArray, viz_output_topic, 10)

    def on_gnss(self, msg: Pose):
        """!
        @brief pose 수신 콜백 — PoseCache 버퍼에 추가.
        @param msg gnss_msgs/msg/Pose 메시지 (hemisphere GNSS 또는 Spatial FOG INS).
        """
        self.pose_cache.update(msg)

    def on_detections(self, msg: Object3DArray):
        """!
        @brief 탐지 결과 수신 시 추적을 수행하고 결과를 두 포맷으로 발행하는 메인 콜백.

        1. **pose 확인**: 아직 pose를 한 번도 못 받았으면 이 프레임은 건너뛴다 (로컬→글로벌 변환이 불가능하므로).
        2. **pose 보간**: 탐지 메시지(= LiDAR 프레임)의 timestamp로 PoseCache에서 두 샘플 사이를 보간한 pose 행렬을 얻는다.
           버퍼 밖으로 pose_max_age_sec 이상 벗어나면(pose 끊김, 시계 불일치) 외삽하지 않고 프레임을 건너뛴다.
        3. **추적**: TrackerAdapter.update()로 프레임 간 ID/속도가 채워진 결과를 얻는다.
        4. **메시지 변환 및 발행**: 같은 추적 결과를 (a) 시각화용 TrackedObject3DArray와 (b) 다른 시스템 호환용
           ObjectsDetected(detect_msgs) 두 포맷으로 동시에 채워 각각 발행한다. ObjectsDetected는 원래 픽셀 좌표 기반
           탐지 스키마라서, `x/y/w/h` 필드에 실제로는 로컬 좌표(m)를 정수로 캐스팅해 채워넣는다 — 필드명이 픽셀을
           뜻하지만 실제 단위는 미터임에 주의.

        @param msg detection_node가 발행한 Object3DArray (로컬 좌표계 기준).
        """
        if not self.pose_cache.has_fix():
            self.get_logger().warning("No GNSS pose received yet, skipping frame.", throttle_duration_sec=5.0)
            return

        boxes, scores, class_ids, class_names = self.object3d_array_to_boxes(msg)
        if len(boxes) == 0:
            return

        query_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose_matrix = self.pose_cache.get_pose_matrix(query_time)
        if pose_matrix is None:
            # 라이다 시각이 pose 버퍼에서 max_age 이상 벗어남 (pose 끊김 / 시계 불일치): 외삽하지 않고 프레임을 건너뛴다
            latest = self.pose_cache.latest_time()
            self.get_logger().warning(
                f"No GNSS pose within {self.pose_cache.max_age_sec}s of the LiDAR stamp "
                f"(lidar - latest pose = {query_time - latest:+.3f}s), skipping frame.", throttle_duration_sec=2.0)
            return

        tracked = self.tracker.update(
            boxes, scores, class_ids, class_names, pose_matrix, query_time
        )

        # --- 시각화용 메시지 ---
        viz_msg = TrackedObject3DArray()
        viz_msg.header = msg.header

        # --- 트래커 출력 메시지 ---
        out_msg = ObjectsDetected()
        out_msg.header = msg.header
        out_msg.number = len(tracked)

        for t in tracked:
            viz_obj = TrackedObject3D()

            viz_obj.detection.center.x, viz_obj.detection.center.y, viz_obj.detection.center.z = t["local_center"]
            viz_obj.detection.size.x, viz_obj.detection.size.y, viz_obj.detection.size.z = t["size"]
            viz_obj.detection.yaw = t["local_yaw"]
            viz_obj.detection.class_id = t["class_id"]
            viz_obj.detection.class_name = t["class_name"]
            viz_obj.detection.score = t["score"]

            viz_obj.track_id = t["track_id"]
            viz_obj.velocity.x, viz_obj.velocity.y, viz_obj.velocity.z = t["velocity"]

            viz_obj.global_position.x, viz_obj.global_position.y, viz_obj.global_position.z = t["global_position"]
            viz_obj.global_yaw = t["global_yaw"]

            viz_msg.objects.append(viz_obj)

            # --- 트래커 출력 채우기 ---
            lx, ly, lz = t["local_center"]
            sx, sy, sz = t["size"]
            vx, vy, vz = t["velocity"]

            obj = InfoDetectedObject()
            obj.id = t["track_id"]
            obj.target_label = t["class_id"]
            obj.score = t["score"]

            obj.x = int(lx)
            obj.y = int(ly)
            obj.w = int(sx)
            obj.h = int(sy)

            obj.range = float(np.hypot(lx, ly))
            obj.theta = float(np.arctan2(ly, lx))

            obj.speed = float(np.hypot(vx, vy))
            obj.heading = float(t["local_yaw"])

            obj.x_covariances = 0.0
            obj.y_covariances = 0.0
            obj.w_covariances = 0.0
            obj.h_covariances = 0.0
            obj.range_covariances = 0.0
            obj.theta_covariances = 0.0
            obj.speed_covariances = 0.0
            obj.heading_covariances = 0.0

            out_msg.d_object.append(obj)

        self.pub.publish(out_msg)
        self.pub_viz.publish(viz_msg)

    def object3d_array_to_boxes(self, msg: Object3DArray):
        """!
        @brief Object3DArray를 Tracker3D/TrackerAdapter가 쓰는 numpy 배열들로 변환 (7열: yaw 까지, roll/pitch 는 추적에 미사용).
        @param msg 변환할 Object3DArray 메시지.
        @return (boxes(N,7) float64, scores(N,) float64, class_ids(N,) int32, class_names(N,) object) 튜플.
        """
        boxes = []
        scores = []
        class_ids = []
        class_names = []

        for obj in msg.objects:
            boxes.append([
                obj.center.x, obj.center.y, obj.center.z,
                obj.size.x, obj.size.y, obj.size.z,
                obj.yaw,
            ])
            scores.append(obj.score)
            class_ids.append(obj.class_id)
            class_names.append(obj.class_name)

        return (
            np.array(boxes, dtype=np.float64),
            np.array(scores, dtype=np.float64),
            np.array(class_ids, dtype=np.int32),
            np.array(class_names, dtype=object),
        )


def main(args=None):
    """!
    @brief ROS2 노드 실행 진입점 (rclpy 초기화 → spin → 종료).
    @param args rclpy.init()에 전달할 커맨드라인 인자.
    """
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()