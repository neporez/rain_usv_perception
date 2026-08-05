import numpy as np
import rclpy
from rclpy.node import Node

from gnss_msgs.msg import Pose
from lidar_perception_interfaces.msg import (
    Object3DArray,
    TrackedObject3D,
    TrackedObject3DArray,
)
from lidar_perception_tracking.pose_cache import PoseCache
from lidar_perception_tracking.tracker_adapter import TrackerAdapter


class TrackingNode(Node):
    def __init__(self):
        super().__init__("tracking_node")

        self.declare_parameter("detection_topic", "/detection/objects")
        self.declare_parameter("output_topic", "/tracking/objects")
        self.declare_parameter("gnss_topic", "/digital_bridge/gnss/pose")
        self.declare_parameter("tracker_config_path", "tracker/config/multi_object_tracker.yaml")

        detection_topic = self.get_parameter("detection_topic").value
        output_topic = self.get_parameter("output_topic").value
        gnss_topic = self.get_parameter("gnss_topic").value
        tracker_config_path = self.get_parameter("tracker_config_path").value

        self.tracker = TrackerAdapter(cfg_path=tracker_config_path)
        self.pose_cache = PoseCache()

        self.gnss_sub = self.create_subscription(
            Pose, gnss_topic, self.on_gnss, 10
        )
        self.detection_sub = self.create_subscription(
            Object3DArray, detection_topic, self.on_detections, 10
        )
        self.pub = self.create_publisher(TrackedObject3DArray, output_topic, 10)

    def on_gnss(self, msg: Pose):
        self.pose_cache.update(msg)

    def on_detections(self, msg: Object3DArray):
        if not self.pose_cache.has_fix():
            self.get_logger().warning("No GNSS fix yet, skipping frame.")
            return

        boxes, scores, class_ids, class_names = self.object3d_array_to_boxes(msg)
        if len(boxes) == 0:
            return

        query_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose_matrix = self.pose_cache.get_pose_matrix(query_time)

        tracked = self.tracker.update(
            boxes, scores, class_ids, class_names, pose_matrix, query_time
        )

        out_msg = TrackedObject3DArray()
        out_msg.header = msg.header

        for t in tracked:
            obj = TrackedObject3D()

            obj.detection.center.x, obj.detection.center.y, obj.detection.center.z = t["local_center"]
            obj.detection.size.x, obj.detection.size.y, obj.detection.size.z = t["size"]
            obj.detection.yaw = t["local_yaw"]
            obj.detection.class_id = t["class_id"]
            obj.detection.class_name = t["class_name"]
            obj.detection.score = t["score"]

            obj.track_id = t["track_id"]
            obj.velocity.x, obj.velocity.y, obj.velocity.z = t["velocity"]

            obj.global_position.x, obj.global_position.y, obj.global_position.z = t["global_position"]
            obj.global_yaw = t["global_yaw"]

            out_msg.objects.append(obj)

        self.pub.publish(out_msg)

    def object3d_array_to_boxes(self, msg: Object3DArray):
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
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
