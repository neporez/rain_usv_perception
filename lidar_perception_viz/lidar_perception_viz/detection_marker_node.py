## @file
#  @brief 탐지 결과(Object3DArray)를 RViz2 MarkerArray 로 그리는 노드 (detection_viz.launch.py).

"""!
@brief Object3DArray (탐지 결과) → visualization_msgs/MarkerArray (RViz2).
상자는 와이어프레임(LINE_LIST)으로 그려 점군을 가리지 않게 하고, 9자유도(yaw, roll, pitch)를 모두 반영한다
(회전 R = Rz(yaw) Ry(pitch) Rx(roll), 데이터셋 / OpenPCDet 규약). 선수 방향은 상자 중심에서 앞면 중심으로 가는 선,
라벨은 클래스 이름 / 점수 / 길이를 상자 위에 띄운다. 색은 기본 초록(예측). 매 프레임 DELETEALL 후 다시 그린다.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from lidar_perception_interfaces.msg import Object3DArray

# 상자 모서리 12개 (꼭짓점 인덱스 쌍). 꼭짓점: x in {+,-} 길이 방향, y in {+,-}, z in {+,-}
_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0),      # 바닥
          (4, 5), (5, 7), (7, 6), (6, 4),      # 천장
          (0, 4), (1, 5), (2, 6), (3, 7)]      # 기둥


def _rotation(yaw, pitch, roll):
    """!
    @brief 상자 회전 행렬 R = Rz(yaw) Ry(pitch) Rx(roll) (데이터셋 / OpenPCDet 규약).
    @param yaw 방위각 [rad]. @param pitch 피치 [rad]. @param roll 롤 [rad].
    @return 3x3 중첩 리스트.
    """
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _corners(obj):
    """!
    @brief Object3D 의 8개 꼭짓점을 센서 좌표계로 계산.
    @param obj lidar_perception_interfaces/Object3D.
    @return ([(x, y, z)] 8개 (인덱스 = ix + 2*iy + 4*iz), 회전 행렬 R).
    """
    R = _rotation(obj.yaw, obj.pitch, obj.roll)
    hx, hy, hz = obj.size.x / 2.0, obj.size.y / 2.0, obj.size.z / 2.0
    cx, cy, cz = obj.center.x, obj.center.y, obj.center.z
    out = []
    for iz in (-1, 1):
        for iy in (-1, 1):
            for ix in (-1, 1):
                lx, ly, lz = ix * hx, iy * hy, iz * hz
                out.append((cx + R[0][0] * lx + R[0][1] * ly + R[0][2] * lz,
                            cy + R[1][0] * lx + R[1][1] * ly + R[1][2] * lz,
                            cz + R[2][0] * lx + R[2][1] * ly + R[2][2] * lz))
    return out, R


class DetectionMarkerNode(Node):
    """!
    @brief 탐지 상자를 초록 와이어프레임(9자유도) + 선수 방향 선 + 라벨 텍스트 마커로 발행.

    매 프레임 DELETEALL 마커를 먼저 넣어 잔상을 없애고, 점수가 min_score 미만인 상자는 그리지 않는다.
    """

    def __init__(self):
        """!
        @brief 파라미터 선언(색, 선 두께, 라벨 등), 구독/발행자 생성.
        """
        super().__init__("detection_marker_node")
        self.declare_parameter("detection_topic", "/detection/objects")
        self.declare_parameter("output_topic", "/viz/detections")
        self.declare_parameter("min_score", 0.0)            # 이 점수 미만은 그리지 않음 (탐지 노드의 score_threshold 와 별개)
        self.declare_parameter("line_width", 0.3)           # m
        self.declare_parameter("color_rgb", [0.0, 1.0, 0.0])  # 예측 = 초록
        self.declare_parameter("show_heading", True)
        self.declare_parameter("show_label", True)
        self.declare_parameter("label_height", 2.0)         # m, 글자 크기
        self.declare_parameter("marker_lifetime_sec", 0.0)  # 0 = 다음 프레임의 DELETEALL 까지 유지

        self.min_score = float(self.get_parameter("min_score").value)
        self.line_width = float(self.get_parameter("line_width").value)
        c = self.get_parameter("color_rgb").value
        self.color = ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0)
        self.show_heading = bool(self.get_parameter("show_heading").value)
        self.show_label = bool(self.get_parameter("show_label").value)
        self.label_height = float(self.get_parameter("label_height").value)
        self.lifetime_sec = float(self.get_parameter("marker_lifetime_sec").value)

        self.sub = self.create_subscription(Object3DArray, self.get_parameter("detection_topic").value, self.on_detections, 10)
        self.pub = self.create_publisher(MarkerArray, self.get_parameter("output_topic").value, 10)

    # ------------------------------------------------------------------
    def _base(self, header, ns, mid, mtype):
        """!
        @brief 공통 필드(header, ns, id, type, 색, lifetime)를 채운 Marker 생성.
        @param header 입력 메시지 header (frame_id, stamp 유지). @param ns 네임스페이스. @param mid 마커 id. @param mtype Marker 종류.
        @return Marker.
        """
        m = Marker()
        m.header = header
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = self.color
        m.lifetime.sec = int(self.lifetime_sec)
        m.lifetime.nanosec = int((self.lifetime_sec % 1.0) * 1e9)
        return m

    def on_detections(self, msg: Object3DArray):
        """!
        @brief 탐지 수신 콜백 — 상자 / 선수 방향 / 라벨 마커를 만들어 MarkerArray 로 발행.
        @param msg detection_node 의 Object3DArray.
        """
        arr = MarkerArray()
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, obj in enumerate(msg.objects):
            if obj.score < self.min_score:
                continue
            corners, R = _corners(obj)

            box = self._base(msg.header, "detection_bbox", i, Marker.LINE_LIST)
            box.scale.x = self.line_width
            for a, b in _EDGES:
                box.points.append(Point(x=corners[a][0], y=corners[a][1], z=corners[a][2]))
                box.points.append(Point(x=corners[b][0], y=corners[b][1], z=corners[b][2]))
            arr.markers.append(box)

            if self.show_heading:
                # 중심 -> 선수(앞면 중심): 상자 길이 축 (+x) 방향
                hx = obj.size.x / 2.0
                head = self._base(msg.header, "detection_heading", i, Marker.LINE_LIST)
                head.scale.x = self.line_width * 1.5
                head.points.append(Point(x=obj.center.x, y=obj.center.y, z=obj.center.z))
                head.points.append(Point(x=obj.center.x + R[0][0] * hx, y=obj.center.y + R[1][0] * hx, z=obj.center.z + R[2][0] * hx))
                arr.markers.append(head)

            if self.show_label:
                txt = self._base(msg.header, "detection_label", i, Marker.TEXT_VIEW_FACING)
                txt.scale.z = self.label_height
                txt.pose.position.x = obj.center.x
                txt.pose.position.y = obj.center.y
                txt.pose.position.z = obj.center.z + obj.size.z / 2.0 + self.label_height
                txt.text = f"{obj.class_name} {obj.score:.2f} | {obj.size.x:.1f} m"
                arr.markers.append(txt)

        self.pub.publish(arr)


def main(args=None):
    """!
    @brief ROS2 노드 실행 진입점 (rclpy 초기화 → spin → 종료).
    @param args rclpy.init()에 전달할 커맨드라인 인자.
    """
    rclpy.init(args=args)
    node = DetectionMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
