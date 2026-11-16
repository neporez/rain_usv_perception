"""
ouster_packet_node.py -- /dev/lidar/lidar_packets (ouster_sensor_msgs/PacketMsg) -> /dev/lidar/points (sensor_msgs/PointCloud2).

실시간 센서나 bag 재생이 원시 패킷만 줄 때 탐지 노드 앞에 두는 노드. 센서 metadata(JSON)는 드라이버가 latched 로
발행하는 /dev/lidar/metadata (std_msgs/String) 또는 metadata_path 파일에서 읽는다. 출력 점군은 드라이버의
/dev/lidar/points 와 같은 좌표계(os_lidar)·필드 이름을 쓰되 유효 점만 담는다 (organized 아님).
bag 재생 시에는 `ros2 bag play --clock` + `use_sim_time:=true` 로 노드 시계를 bag 시각에 맞춰야 GNSS pose 와 시각이 맞는다.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
from ouster_sensor_msgs.msg import PacketMsg

from lidar_perception_detection.ouster_packets import OusterPacketDecoder

_FIELDS = [("x", PointField.FLOAT32, np.float32), ("y", PointField.FLOAT32, np.float32), ("z", PointField.FLOAT32, np.float32),
           ("intensity", PointField.FLOAT32, np.float32), ("reflectivity", PointField.UINT16, np.uint16),
           ("ring", PointField.UINT16, np.uint16), ("range", PointField.UINT32, np.uint32)]


class OusterPacketNode(Node):
    def __init__(self):
        super().__init__("ouster_packet_node")
        self.declare_parameter("packets_topic", "/dev/lidar/lidar_packets")
        self.declare_parameter("metadata_topic", "/dev/lidar/metadata")
        self.declare_parameter("metadata_path", "")              # JSON 파일 (토픽이 없을 때)
        self.declare_parameter("output_topic", "/dev/lidar/points")
        self.declare_parameter("frame_id", "os_lidar")
        self.declare_parameter("point_frame", "lidar")           # 'lidar' = os_lidar (드라이버와 동일) | 'sensor' = os_sensor
        self.declare_parameter("stamp_mode", "driver")             # 'driver' = ouster_ros 와 동일(직전 프레임 첫 패킷 시각) | 'scan_start'
        self.declare_parameter("packet_qos_depth", 4096)
        self.decoder = None
        self.frame_id = self.get_parameter("frame_id").value
        self.point_frame = self.get_parameter("point_frame").value
        self.stamp_mode = self.get_parameter("stamp_mode").value
        self._dtype = np.dtype([(n, t) for n, _, t in _FIELDS])
        self._fields = []
        off = 0
        for n, pf_t, t in _FIELDS:
            self._fields.append(PointField(name=n, offset=off, datatype=pf_t, count=1)); off += np.dtype(t).itemsize

        path = self.get_parameter("metadata_path").value
        if path:
            with open(path) as f:
                self._init_decoder(f.read(), "file " + path)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.meta_sub = self.create_subscription(String, self.get_parameter("metadata_topic").value, self.on_metadata, latched)
        pkt_qos = QoSProfile(depth=int(self.get_parameter("packet_qos_depth").value), reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.pkt_sub = self.create_subscription(PacketMsg, self.get_parameter("packets_topic").value, self.on_packet, pkt_qos)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self._dropped = 0
        self.get_logger().info(f"packets {self.get_parameter('packets_topic').value} -> {self.get_parameter('output_topic').value} "
                               f"(frame_id {self.frame_id}); waiting for metadata" if self.decoder is None else "decoder ready: " + self.decoder.describe())

    def _init_decoder(self, json_str, source):
        if self.decoder is not None:
            return
        self.decoder = OusterPacketDecoder(json_str, point_frame=self.point_frame, stamp_mode=self.stamp_mode)
        self.get_logger().info(f"metadata from {source}: {self.decoder.describe()}")

    def on_metadata(self, msg: String):
        try:
            self._init_decoder(msg.data, "topic")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"bad metadata: {e}")

    def on_packet(self, msg: PacketMsg):
        if self.decoder is None:
            self._dropped += 1
            if self._dropped % 1280 == 1:
                self.get_logger().warning("no sensor metadata yet (topic %s or metadata_path) -- dropping packets" % self.get_parameter("metadata_topic").value)
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns == 0:          # use_sim_time and no /clock received yet: a frame stamped 0 would break the pose lookup
            return
        out = self.decoder.feed(msg.buf, now_ns)
        if out is None:
            return
        n = out["xyz"].shape[0]
        arr = np.empty(n, dtype=self._dtype)
        arr["x"], arr["y"], arr["z"] = out["xyz"][:, 0], out["xyz"][:, 1], out["xyz"][:, 2]
        arr["intensity"] = out["intensity"]; arr["reflectivity"] = out["reflectivity"]; arr["ring"] = out["ring"]; arr["range"] = out["range"]
        pc = PointCloud2()
        pc.header.frame_id = self.frame_id
        pc.header.stamp.sec = out["stamp_ns"] // 1_000_000_000
        pc.header.stamp.nanosec = out["stamp_ns"] % 1_000_000_000
        pc.height, pc.width = 1, n
        pc.fields = self._fields
        pc.is_bigendian = False
        pc.point_step = self._dtype.itemsize
        pc.row_step = pc.point_step * n
        pc.is_dense = True
        pc.data = arr.tobytes()
        self.pub.publish(pc)
        if self.decoder.n_frames % 100 == 1:
            self.get_logger().info(f"frame {self.decoder.n_frames}: {n} points, {out['n_valid_cols']} valid columns, bad-size packets {self.decoder.n_bad_size}")


def main(args=None):
    rclpy.init(args=args)
    node = OusterPacketNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
