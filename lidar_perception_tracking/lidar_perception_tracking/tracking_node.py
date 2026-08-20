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
    def __init__(self):
        super().__init__("tracking_node")

        self.declare_parameter("detection_topic", "/detection/objects")
        self.declare_parameter("output_topic", "/alg1/dl_pst_detection")
        self.declare_parameter("viz_output_topic", "/tracking/objects_viz")
        self.declare_parameter("gnss_topic", "/digital_bridge/gnss/pose")
        self.declare_parameter("tracker_config_path", "tracker/config/multi_object_tracker.yaml")

        detection_topic = self.get_parameter("detection_topic").value
        output_topic = self.get_parameter("output_topic").value
        viz_output_topic = self.get_parameter("viz_output_topic").value
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

        # 트래커 출력 (detect_msgs 포맷)
        self.pub = self.create_publisher(ObjectsDetected, output_topic, 10)
        # 시각화용 출력 (기존 TrackedObject3DArray 포맷)
        self.pub_viz = self.create_publisher(TrackedObject3DArray, viz_output_topic, 10)

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