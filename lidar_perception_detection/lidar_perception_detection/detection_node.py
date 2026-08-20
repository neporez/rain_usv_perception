import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from lidar_perception_interfaces.msg import Object3D, Object3DArray
from lidar_perception_detection.detection_utils import resolve_qos, parse_pointcloud2
from lidar_perception_detection.model_adapter import ModelInferencer

class DetectionNode(Node):
    def __init__(self):
        super().__init__("lidar_detection_node")

        self.declare_parameter("pointcloud_topic", "/dev/lidar/points")
        self.declare_parameter("pointcloud_qos_reliability", "best_effort")
        self.declare_parameter("model_config_path", "cfgs/kriso_models/cbgs_voxel01_voxelnext.yaml")
        self.declare_parameter("ckpt_path", "cfgs/ckpt/voxelnext_epoch_160.pth")
        self.declare_parameter("score_threshold", 0.3)
        self.declare_parameter("output_topic", "/detection/objects")

        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        qos_reliability = self.get_parameter("pointcloud_qos_reliability").value
        model_config_path = self.get_parameter("model_config_path").value
        ckpt_path = self.get_parameter("ckpt_path").value
        self.score_threshold = self.get_parameter("score_threshold").value
        output_topic = self.get_parameter("output_topic").value

        self.get_logger().info("Loading model...")
        self.inferencer = ModelInferencer(
            cfg_path=model_config_path,
            ckpt_path=ckpt_path,
            score_thresh=self.score_threshold
        )
        self.get_logger().info("Model loaded.")

        qos = resolve_qos(qos_reliability, depth=1)
        self.sub = self.create_subscription(
            PointCloud2, pointcloud_topic, self.lidar_callback, qos
        )
        self.pub = self.create_publisher(Object3DArray, output_topic, 10)

    def lidar_callback(self, msg: PointCloud2):

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
            obj.class_id = det["class_id"]
            obj.class_name = det["class_name"]
            obj.score = det["score"]
            out_msg.objects.append(obj)

        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()