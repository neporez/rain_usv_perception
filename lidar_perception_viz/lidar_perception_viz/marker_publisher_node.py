import colorsys

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from lidar_perception_interfaces.msg import TrackedObject3DArray


def track_id_to_color(track_id: int):
    """
    track_id를 hash해서 고정된 hue 값을 얻고, HSV -> RGB로 변환.
    같은 track_id는 항상 같은 색을 갖도록 보장.
    """
    hue = (track_id * 0.61803398875) % 1.0 
    r, g, b = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
    return r, g, b


class MarkerPublisherNode(Node):
    def __init__(self):
        super().__init__("marker_publisher_node")

        self.declare_parameter("tracking_topic", "/tracking/objects")
        self.declare_parameter("output_topic", "/viz/tracks")

        tracking_topic = self.get_parameter("tracking_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._prev_track_ids = set()

        self.sub = self.create_subscription(
            TrackedObject3DArray, tracking_topic, self.on_tracks, 10
        )
        self.pub = self.create_publisher(MarkerArray, output_topic, 10)

    def on_tracks(self, msg: TrackedObject3DArray):
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
        import math
        return math.sin(x)

    @staticmethod
    def _cos(x):
        import math
        return math.cos(x)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()