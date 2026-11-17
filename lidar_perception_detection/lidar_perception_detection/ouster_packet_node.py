## @file
#  @brief Ouster 원시 패킷 토픽을 PointCloud2 로 변환해 발행하는 ROS2 노드 (input:=packets).

"""!
@brief /dev/lidar/lidar_packets (ouster_sensor_msgs/PacketMsg) → /dev/lidar/points (sensor_msgs/PointCloud2) 변환 노드.

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
    """!
    @brief 원시 패킷 구독 → OusterPacketDecoder → PointCloud2 발행.

    센서 metadata 는 드라이버의 latched 토픽(metadata_topic) 또는 파일(metadata_path)에서 받으며, 받기 전의 패킷은
    버린다(경고). 출력은 드라이버의 /dev/lidar/points 와 같은 좌표계(os_lidar)와 필드 이름(x, y, z, intensity,
    reflectivity, ring, range)을 쓰되 유효 점만 담는다. use_sim_time 일 때 /clock 을 받기 전에는 프레임을 내보내지 않는다.

    주요 구성:
    - on_metadata(): metadata 수신 → 디코더 생성.
    - on_packet(): 패킷 1개 → 디코더 → 프레임 완성 시 PointCloud2 발행.
    """

    def __init__(self):
        """!
        @brief 파라미터 선언, 디코더(파일 metadata 가 있으면 즉시) 준비, 구독/발행자 생성.
        """
        super().__init__("ouster_packet_node")
        self.declare_parameter("packets_topic", "/dev/lidar/lidar_packets")
        self.declare_parameter("metadata_topic", "/dev/lidar/metadata")
        self.declare_parameter("metadata_path", "")              # JSON 파일 (토픽이 없을 때)
        self.declare_parameter("output_topic", "/dev/lidar/points")
        self.declare_parameter("frame_id", "os_lidar")
        self.declare_parameter("point_frame", "lidar")           # 'lidar' = os_lidar (드라이버와 동일) | 'sensor' = os_sensor
        self.declare_parameter("stamp_mode", "driver")             # 'driver' = ouster_ros 와 동일(직전 프레임 첫 패킷 시각) | 'scan_start'
        self.declare_parameter("packet_qos_depth", 4096)
        self.declare_parameter("output_qos_reliability", "reliable")   # 700 KB 점군을 best_effort 로 내면 FastDDS 3 (Lyrical) 에서 같은 PC 안에서도 90 % 가 버려짐
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
        out_rel = QoSReliabilityPolicy.RELIABLE if self.get_parameter("output_qos_reliability").value == "reliable" else QoSReliabilityPolicy.BEST_EFFORT
        # reliable publisher + best_effort subscriber(탐지 노드 기본값)는 QoS 호환이므로 구독 쪽 설정은 그대로 둘 수 있다
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, QoSProfile(depth=5, reliability=out_rel))
        self._dropped = 0
        self.get_logger().info(f"packets {self.get_parameter('packets_topic').value} -> {self.get_parameter('output_topic').value} "
                               f"(frame_id {self.frame_id}); waiting for metadata" if self.decoder is None else "decoder ready: " + self.decoder.describe())

    def _init_decoder(self, json_str, source):
        """!
        @brief metadata JSON 으로 디코더를 한 번만 생성.
        @param json_str 센서 metadata JSON.
        @param source 로그용 출처 문자열 ("topic" 또는 파일 경로).
        """
        if self.decoder is not None:
            return
        self.decoder = OusterPacketDecoder(json_str, point_frame=self.point_frame, stamp_mode=self.stamp_mode)
        self.get_logger().info(f"metadata from {source}: {self.decoder.describe()}")

    def on_metadata(self, msg: String):
        """!
        @brief latched metadata 토픽 콜백.
        @param msg std_msgs/String (센서 metadata JSON).
        """
        try:
            self._init_decoder(msg.data, "topic")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"bad metadata: {e}")

    def on_packet(self, msg: PacketMsg):
        """!
        @brief 라이다 패킷 콜백 — 디코더에 넣고 프레임이 완성되면 PointCloud2 로 발행.
        @param msg ouster_sensor_msgs/PacketMsg (buf = 패킷 바이트열).
        """
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
    """!
    @brief ROS2 노드 실행 진입점 (rclpy 초기화 → spin → 종료).
    @param args rclpy.init()에 전달할 커맨드라인 인자.
    """
    rclpy.init(args=args)
    node = OusterPacketNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
