## @file
#  @brief 추적 결과를 RViz MarkerArray로 발행하는 노드와 색상 매핑 유틸.

"""! @brief 추적 결과를 RViz MarkerArray로 발행하는 노드와 색상 매핑 유틸. """
import colorsys

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from lidar_perception_interfaces.msg import TrackedObject3DArray


def track_id_to_color(track_id: int):
    """!
    @brief track_id를 고정된 색으로 매핑 (같은 ID는 항상 같은 색).

    황금비의 켤레(0.61803398875)를 hue에 곱해 mod 1.0을 취하는 방식으로,
    연속된 track_id끼리도 색상환에서 최대한 멀리 떨어지도록(비슷한 색이
    연달아 나오지 않도록) 분산시킨다.

    @param track_id 색을 얻을 트랙 ID.
    @return (r, g, b) — 0~1 범위의 RGB 튜플.
    """
    hue = (track_id * 0.61803398875) % 1.0 
    r, g, b = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
    return r, g, b


class MarkerPublisherNode(Node):
    """!
    @brief TrackedObject3DArray를 RViz용 MarkerArray(CUBE)로 변환해 발행하는 노드.

    매 프레임 현재 트랙들의 bbox 마커를 발행하는 것뿐 아니라, 직전 프레임에는
    있었지만 이번 프레임에 사라진 track_id에 대해 DELETE 마커를 추가로 발행해서
    RViz 화면에 사라진 트랙의 잔상이 남지 않도록 처리한다.
    """

    def __init__(self):
        """! @brief 파라미터 선언, 구독/발행자 생성. """
        super().__init__("marker_publisher_node")

        self.declare_parameter("tracking_viz_topic", "/tracking/objects_viz")
        self.declare_parameter("output_topic", "/viz/tracks")

        tracking_topic = self.get_parameter("tracking_viz_topic").value
        output_topic = self.get_parameter("output_topic").value

        ## 직전 프레임에 발행했던 track_id 집합 (사라진 트랙 DELETE 마커 계산용)
        self._prev_track_ids = set()

        ## 추적 결과(TrackedObject3DArray) 구독자
        self.sub = self.create_subscription(
            TrackedObject3DArray, tracking_topic, self.on_tracks, 10
        )
        ## RViz 시각화용 MarkerArray 발행자
        self.pub = self.create_publisher(MarkerArray, output_topic, 10)

    def on_tracks(self, msg: TrackedObject3DArray):
        """!
        @brief 추적 결과 수신 콜백 — 현재 트랙 마커 생성 + 사라진 트랙 DELETE 마커 추가.
        @param msg tracking_node가 발행한 TrackedObject3DArray.
        """
        marker_array = MarkerArray()
        current_ids = set()

        for obj in msg.objects:
            marker = self.make_bbox_marker(obj, msg.header)
            marker_array.markers.append(marker)
            current_ids.add(obj.track_id)

        disappeared_ids = self._prev_track_ids - current_ids
        for track_id in disappeared_ids:
            delete_marker = Marker()
            delete_marker.header = msg.header
            delete_marker.ns = "tracked_bbox"
            delete_marker.id = track_id
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

        self._prev_track_ids = current_ids
        self.pub.publish(marker_array)

    def make_bbox_marker(self, obj, header) -> Marker:
        """!
        @brief TrackedObject3D 하나를 RViz CUBE Marker로 변환.

        yaw는 z축 회전만 있는 평면 회전이라고 가정하고 quaternion으로 변환한다
        (x=y=0, z=sin(yaw/2), w=cos(yaw/2)). lifetime을 0으로 두어 마커가
        자동 만료되지 않게 하고, 대신 on_tracks()가 사라진 트랙을 명시적으로
        DELETE 처리한다.

        @param obj 마커로 그릴 TrackedObject3D.
        @param header 마커에 사용할 header (프레임/타임스탬프).
        @return 구성된 visualization_msgs/Marker (CUBE, ADD).
        """
        marker = Marker()
        marker.header = header
        marker.ns = "tracked_bbox"
        marker.id = obj.track_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = obj.detection.center.x
        marker.pose.position.y = obj.detection.center.y
        marker.pose.position.z = obj.detection.center.z

        # yaw -> quaternion (z축 회전만 있는 평면 회전)
        half_yaw = obj.detection.yaw * 0.5
        marker.pose.orientation.z = self._sin(half_yaw)
        marker.pose.orientation.w = self._cos(half_yaw)

        marker.scale.x = obj.detection.size.x
        marker.scale.y = obj.detection.size.y
        marker.scale.z = obj.detection.size.z

        r, g, b = track_id_to_color(obj.track_id)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.6

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        return marker

    @staticmethod
    def _sin(x):
        """!
        @brief math.sin 래퍼 (quaternion 변환용).
        @param x 라디안 각도.
        """
        import math
        return math.sin(x)

    @staticmethod
    def _cos(x):
        """!
        @brief math.cos 래퍼 (quaternion 변환용).
        @param x 라디안 각도.
        """
        import math
        return math.cos(x)


def main(args=None):
    """!
    @brief ROS2 노드 실행 진입점 (rclpy 초기화 → spin → 종료).
    @param args rclpy.init()에 전달할 커맨드라인 인자.
    """
    rclpy.init(args=args)
    node = MarkerPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()